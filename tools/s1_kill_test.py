#!/usr/bin/env python3
"""S1 — kill switch (RC ch8) while armed in AutoNav. THE safety backstop test.

Settles a documented contradiction: rover_autonav_collision_stop.md says the ch8
kill is "proven to work armed inside AutoNav"; autonomy_plan.md and
autonav_reference.md §12 record S1 as UNTESTED and gating every armed autonomous
drive. Both cannot be true.

THIS MOVES A LIVE VEHICLE. Clear run-out, hand ON the kill switch, on the FLOOR.

Never arms and never disarms. YOU arm via RC beforehand; YOU kill with ch8. The
script only switches mode, commands a slow creep, and measures what happens.

  python3 tools/s1_kill_test.py                 # 0.15 m/s, 12 s bound
  python3 tools/s1_kill_test.py --speed 0.10

Pass criterion: on ch8, wheels stop immediately and the vehicle disarms.
Measured: latency from the disarm transition to all-wheel RPM == 0.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import EscStatus, VehicleCommand, VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

ARM_ARMED, NAV_MANUAL, NAV_AUTONAV = 2, 0, 23


class S1(Node):
    def __init__(self, speed, bound):
        super().__init__('s1_kill_test')
        self.speed, self.bound = speed, bound
        self.arming = None
        self.nav = None
        self.rpm = {}
        self.rpm_t = 0.0
        self.log = []
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.status_cb, PX4_QOS)
        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc_cb, PX4_QOS)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vcmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

    def status_cb(self, m):
        self.arming, self.nav = m.arming_state, m.nav_state

    def esc_cb(self, m):
        self.rpm = {e.esc_address: e.esc_rpm for e in m.esc[:m.esc_count]}
        self.rpm_t = time.time()

    def max_rpm(self):
        return max((abs(v) for v in self.rpm.values()), default=0)

    def set_mode(self, main, sub):
        v = VehicleCommand()
        v.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        v.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        v.param1, v.param2, v.param3 = 1.0, float(main), float(sub)
        v.target_system, v.target_component = 1, 1
        v.source_system, v.source_component = 1, 1
        v.from_external = True
        self.vcmd.publish(v)

    def drive(self, vx):
        t = Twist()
        t.linear.x = float(vx)
        self.cmd.publish(t)

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--bound', type=float, default=12.0, help='max seconds of motion')
    a = ap.parse_args()

    rclpy.init()
    n = S1(a.speed, a.bound)
    n.spin(6.0)

    if n.arming is None:
        print('no vehicle_status — is microxrce-agent up?'); rclpy.shutdown(); sys.exit(1)
    print(f'state: arming={n.arming} nav={n.nav} wheels_rpm_max={n.max_rpm()}')
    if n.arming != ARM_ARMED:
        print('NOT ARMED. Arm via RC first. This script never arms.'); rclpy.shutdown(); sys.exit(1)
    if n.max_rpm() != 0:
        print('WHEELS ARE TURNING. Aborting.'); rclpy.shutdown(); sys.exit(1)

    print('switching to AutoNav (main=4 sub=11) ...')
    for _ in range(5):
        n.set_mode(4, 11); n.spin(0.4)
    if n.nav != NAV_AUTONAV:
        print(f'FAILED to enter AutoNav (nav_state={n.nav}). No motion. Aborting.')
        rclpy.shutdown(); sys.exit(1)
    n.spin(6.0)
    if n.nav != NAV_AUTONAV:
        print(f'AutoNav did NOT HOLD (nav_state={n.nav}). Aborting.'); rclpy.shutdown(); sys.exit(1)
    print(f'AutoNav holding (nav_state={n.nav}).')

    print(f'\n>>> DRIVING at {a.speed} m/s — HIT CH8 NOW <<<\n')
    t0 = time.time()
    t_kill = None
    moved = 0
    while time.time() - t0 < a.bound:
        if t_kill is None:
            n.drive(a.speed)
        rclpy.spin_once(n, timeout_sec=0.02)
        el = time.time() - t0
        r = n.max_rpm()
        if r > 0:
            moved += 1
        n.log.append((el, n.arming, n.nav, r))
        if t_kill is None and n.arming is not None and n.arming != ARM_ARMED:
            t_kill = el
            print(f'*** DISARM DETECTED at t={el:.3f} s (arming_state={n.arming}) — '
                  f'stopping /cmd_vel ***')
        if t_kill is not None and el - t_kill > 3.0:
            break

    n.drive(0.0); n.spin(0.5); n.drive(0.0)

    print('\n================ S1 RESULT ================')
    if t_kill is None:
        print('NO DISARM SEEN within the time bound.')
        print(f'  wheels turned during {moved} samples; final rpm {n.max_rpm()}')
        print('  => ch8 did NOT disarm. S1 FAILS as specified. Kill power manually.')
    else:
        after = [(t, r) for (t, ar, nv, r) in n.log if t >= t_kill]
        stop_t = next((t for t, r in after if r == 0), None)
        peak = max((r for (_, _, _, r) in n.log if _ < t_kill), default=0) \
            if False else max((r for (t, _, _, r) in n.log if t < t_kill), default=0)
        print(f'  disarm at        t = {t_kill:.3f} s')
        print(f'  peak wheel rpm before kill = {peak}')
        if stop_t is None:
            print('  🔴 WHEELS NEVER READ ZERO after the disarm — investigate.')
        else:
            print(f'  wheels zero at   t = {stop_t:.3f} s')
            print(f'  ==> KILL LATENCY = {1000*(stop_t - t_kill):.0f} ms')
            print('  ✅ S1 PASSES: ch8 disarmed and the wheels stopped.'
                  if peak > 0 else
                  '  ⚠️ wheels never turned before the kill — inconclusive, re-run with more speed.')
    print('===========================================')
    print(f'final: arming={n.arming} nav={n.nav} rpm={n.max_rpm()}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
