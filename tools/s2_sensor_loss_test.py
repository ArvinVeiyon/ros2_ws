#!/usr/bin/env python3
"""S2 — sensor loss while driving in AutoNav. Does the reflex fail SAFE?

Pass criterion (autonomy_plan.md): forward blocked within 0.5 s of losing /scan.
The reflex's own scan_timeout is 0.5 s, so a correct result lands just above it -
report the measurement, do not round it to a verdict.

THIS MOVES A LIVE VEHICLE. Clear run-out, hand ON the kill switch (ch8), FLOOR only.
If the reflex does NOT fail safe, the rover keeps driving and only you stop it.

Never arms, never disarms. Requires rover-ekf-bridge running (setup_manual C10.3).
The scan publisher is killed with a plain SIGTERM - it runs as this user.

/scan DOES NOT SELF-RESTORE. This file used to claim it did, via rover-scan's
Restart=always; that is wrong and it cost ten minutes of a dead sensor on
2026-08-11. The unit launches TWO children - a static_transform_publisher and
depthimage_to_laserscan_node - so killing the scan node leaves the launch (the
unit's MAIN process) alive holding the surviving TF child. systemd sees a
healthy unit, Restart never fires, and `systemctl is-active` says active while
/scan is silent. Run `sudo systemctl restart rover-scan` afterwards, every time,
and verify /scan is actually publishing before trusting anything downstream.

  python3 tools/s2_sensor_loss_test.py                # 0.15 m/s
  python3 tools/s2_sensor_loss_test.py --settle 3.0   # drive longer before the cut
"""
import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import EscStatus, RoverSpeedSetpoint, VehicleCommand, VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
# /fmu/in/* are published by px4_ros2 INSIDE the mode, not by the DDS agent, and
# they are VOLATILE - unlike /fmu/out/*, which the agent publishes TRANSIENT_LOCAL.
# Subscribing TRANSIENT_LOCAL here receives NOTHING while looking exactly like a
# working subscription on a quiet topic. See tools/s2_stands_test.py.
SETPOINT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
ARM_ARMED, NAV_AUTONAV = 2, 23


class S2(Node):
    def __init__(self):
        super().__init__('s2_sensor_loss')
        self.arming = self.nav = None
        self.rpm = {}
        self.last_scan = None
        self.sp = None            # last speed the mode actually emitted
        self.sp_n = 0
        self.sp_zero_at = None    # wall time the emitted speed first hit zero
        self.watch_sp = False     # only latch after the cut
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.st_cb, PX4_QOS)
        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc_cb, PX4_QOS)
        self.create_subscription(RoverSpeedSetpoint, '/fmu/in/rover_speed_setpoint',
                                 self.sp_cb, SETPOINT_QOS)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vcmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

    def st_cb(self, m):
        self.arming, self.nav = m.arming_state, m.nav_state

    def esc_cb(self, m):
        self.rpm = {e.esc_address: e.esc_rpm for e in m.esc[:m.esc_count]}

    def sp_cb(self, m):
        self.sp = m.speed_body_x
        self.sp_n += 1
        if self.watch_sp and self.sp_zero_at is None and abs(self.sp) <= 0.001:
            self.sp_zero_at = time.time()

    def scan_cb(self, _):
        self.last_scan = time.time()

    def max_rpm(self):
        return max((abs(v) for v in self.rpm.values()), default=0)

    def set_mode(self, main, sub):
        v = VehicleCommand()
        v.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        v.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        v.param1, v.param2, v.param3 = 1.0, float(main), float(sub)
        v.target_system = v.target_component = 1
        v.source_system = v.source_component = 1
        v.from_external = True
        self.vcmd.publish(v)

    def drive(self, vx):
        t = Twist(); t.linear.x = float(vx); self.cmd.publish(t)

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)


def scan_pid():
    r = subprocess.run(['pgrep', '-f', 'depthimage_to_laserscan_node'],
                       capture_output=True, text=True)
    pids = [int(x) for x in r.stdout.split()]
    return pids[-1] if pids else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--settle', type=float, default=2.5, help='seconds of driving before the cut')
    ap.add_argument('--bound', type=float, default=6.0, help='max seconds after the cut')
    a = ap.parse_args()

    pid = scan_pid()
    if pid is None:
        print('no depthimage_to_laserscan process — is rover-scan up?'); sys.exit(1)

    rclpy.init(); n = S2(); n.spin(6.0)
    if n.arming is None:
        print('no vehicle_status'); rclpy.shutdown(); sys.exit(1)
    if n.last_scan is None:
        print('no /scan arriving — nothing to lose'); rclpy.shutdown(); sys.exit(1)
    print(f'state: arming={n.arming} nav={n.nav} rpm={n.max_rpm()} scan=OK scan_pid={pid}')
    if n.arming != ARM_ARMED:
        print('NOT ARMED. Arm via RC first; this script never arms.'); rclpy.shutdown(); sys.exit(1)
    if n.max_rpm() != 0:
        print('WHEELS TURNING. Aborting.'); rclpy.shutdown(); sys.exit(1)

    print('switching to AutoNav ...')
    for _ in range(5):
        n.set_mode(4, 11); n.spin(0.4)
    n.spin(2.0)
    if n.nav != NAV_AUTONAV:
        print(f'AutoNav did not hold (nav={n.nav}). No motion. Aborting.')
        rclpy.shutdown(); sys.exit(1)
    print(f'AutoNav holding (nav={n.nav}).')

    print(f'\n>>> DRIVING at {a.speed} m/s for {a.settle}s, then CUTTING /scan <<<')
    print('    (hand on ch8 — if the reflex fails, only you stop it)\n')
    t0 = time.time()
    while time.time() - t0 < a.settle:
        n.drive(a.speed); rclpy.spin_once(n, timeout_sec=0.01)
    peak = n.max_rpm()
    if peak == 0:
        n.drive(0.0); n.spin(0.5)
        print(f'WHEELS NEVER TURNED (rpm 0) — reflex may already be blocking. Inconclusive.')
        print('Check: journalctl -u rover-autonav-mode | grep collision-diag')
        rclpy.shutdown(); sys.exit(1)
    print(f'    moving, peak rpm {peak}')

    # INSTRUMENT CHECK, and it must happen HERE - while driving, before the cut.
    # A subscription that receives nothing would later latch no zero at all and
    # read as "the setpoint never dropped", and a channel stuck at zero would
    # read as an instant reflex. Neither is distinguishable from a real result
    # without proving the topic carries the NON-ZERO commanded value right now.
    if n.sp_n == 0:
        n.drive(0.0); n.spin(0.5)
        print('ABORT: no messages on /fmu/in/rover_speed_setpoint while driving.')
        print('       The instrument is broken - this is NOT a reflex measurement.')
        rclpy.shutdown(); sys.exit(2)
    if n.sp is None or abs(n.sp) <= 0.001:
        n.drive(0.0); n.spin(0.5)
        print(f'ABORT: setpoint reads {n.sp} while the wheels turn at {peak} rpm.')
        print('       A channel already at zero cannot demonstrate a drop to zero.')
        rclpy.shutdown(); sys.exit(2)
    print(f'    setpoint instrument OK: {n.sp_n} msgs, emitting {n.sp:.3f} m/s')

    n.watch_sp = True             # latch the first zero from here on, not before
    os.kill(pid, signal.SIGTERM)
    t_cut = time.time()
    print(f'*** /scan publisher killed (pid {pid}) at t={t_cut-t0:.3f}s ***')

    t_stop = None
    last_scan_at_cut = n.last_scan
    while time.time() - t_cut < a.bound:
        n.drive(a.speed)                      # keep commanding: the reflex must override us
        rclpy.spin_once(n, timeout_sec=0.01)
        if t_stop is None and n.max_rpm() == 0:
            t_stop = time.time()
            break
    n.drive(0.0); n.spin(0.5); n.drive(0.0)

    final_scan = n.last_scan
    print('\n================ S2 RESULT ================')
    print(f'  peak rpm while driving      {peak}')
    print(f'  last /scan before the cut   t={last_scan_at_cut-t0:.3f}s')
    print(f'  publisher killed            t={t_cut-t0:.3f}s')
    if n.sp_zero_at is not None:
        print(f'  SETPOINT zeroed             t={n.sp_zero_at-t0:.3f}s')
    else:
        print(f'  SETPOINT never zeroed       (last emitted {n.sp})')

    if t_stop is None:
        print(f'  🔴 WHEELS STILL TURNING after {a.bound:.1f}s — rpm {n.max_rpm()}')
        print('  => S2 FAILS: forward was NOT blocked on sensor loss.')
    else:
        lat_cut = t_stop - t_cut
        lat_scan = t_stop - final_scan
        print(f'  wheels zero                 t={t_stop-t0:.3f}s')
        print(f'  ==> latency from KILL       {1000*lat_cut:.0f} ms')
        print(f'  ==> latency from LAST SCAN  {1000*lat_scan:.0f} ms   '
              f'(reflex scan_timeout = 500 ms)')

    # THE SPLIT. "last scan -> wheels zero" is two different things end to end:
    # the reflex DECIDING (bounded by scan_timeout) and the wheels physically
    # SPINNING DOWN afterwards. They have opposite fixes - a slow decision means
    # scan_timeout or the update rate; a slow spin-down means a speed limit or
    # braking - so a single combined number cannot tell you what to change.
    if n.sp_zero_at is not None:
        decide = n.sp_zero_at - final_scan
        print(f'\n  --- decision vs mechanics ---')
        print(f'  DECIDE  last scan -> setpoint zero   {1000*decide:.0f} ms   '
              f'(scan_timeout = 500 ms)')
        if t_stop is not None:
            spin = t_stop - n.sp_zero_at
            print(f'  COAST   setpoint zero -> rpm zero    {1000*spin:.0f} ms   '
                  f'(mechanical, not the reflex)')
        ok = decide <= 0.8
        print(f'  {"✅" if ok else "⚠️"} REFLEX {"PASSES" if ok else "SLOW"} on decision latency.')
    print('===========================================')
    print(f'final: arming={n.arming} nav={n.nav} rpm={n.max_rpm()}')
    print('\n⚠️  /scan IS STILL DEAD — it does NOT self-restore (see the module docstring).')
    print('    sudo systemctl restart rover-scan     then verify /scan is publishing.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
