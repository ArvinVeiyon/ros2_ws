#!/usr/bin/env python3
"""S2 stands phase — does the reflex ACT, not just sense?

A1 (2026-08-10) validated the perception-health gate's SENSING: the diag timer
reported BLIND correctly under a covered lens. It did not validate the ACTING
path, because the mode was inactive, so forwardBlocked() never gated a real
setpoint. This test closes exactly that gap, and nothing more.

WHAT IT DOES
  Engages AutoNav **DISARMED**, commands a steady forward /cmd_vel, and watches
  the mode's real output on /fmu/in/rover_speed_setpoint while the operator
  covers the camera. Pass = the commanded speed is forced to 0 while blind.

WHY DISARMED IS THE RIGHT CHOICE ON STANDS
  Disarmed, PX4 relaxes mode requirements and accepts AutoNav without a velocity
  estimate, so rover-ekf-bridge is NOT needed - which matters, because bridge +
  wheels-up is the self-sustaining limit-cycle hazard (setup_manual C10). And
  disarmed the motors cannot turn at all, so a reflex failure costs nothing.
  The cost of disarming: this proves the setpoint is zeroed, not that the wheels
  stop. The floor phase (tools/s2_sensor_loss_test.py) proves the rest.

NEVER ARMS, NEVER DISARMS. Aborts if it ever observes ARMED.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import RoverSpeedSetpoint, VehicleCommand, VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
# /fmu/in/* are published by px4_ros2 inside the mode, NOT by the DDS agent, and
# they are VOLATILE. Subscribing with TRANSIENT_LOCAL silently receives NOTHING -
# rclpy logs one incompatible-QoS warning and then behaves exactly like a working
# subscription with a quiet topic. That produced a 70 s run of "NO SETPOINT SEEN"
# on 2026-08-10 that looked like a reflex result and was pure instrument error.
SETPOINT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
ARM_ARMED, NAV_AUTONAV = 2, 23
MAIN_CUSTOM, SUB_AUTONAV = 4, 11
SECTOR_HALF = 0.35


class S2Stands(Node):
    def __init__(self, speed):
        super().__init__('s2_stands')
        self.speed = speed
        self.arming = self.nav = None
        self.sp = None            # last commanded speed the mode actually emitted
        self.sp_n = 0
        self.valid = None
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.st_cb, PX4_QOS)
        self.create_subscription(RoverSpeedSetpoint, '/fmu/in/rover_speed_setpoint',
                                 self.sp_cb, SETPOINT_QOS)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vcmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

    def st_cb(self, m):
        self.arming, self.nav = m.arming_state, m.nav_state

    def sp_cb(self, m):
        self.sp = m.speed_body_x
        self.sp_n += 1

    def scan_cb(self, m):
        n = len(m.ranges)
        if not n:
            self.valid = 0.0
            return
        v = sum(1 for r in m.ranges
                if math.isfinite(r) and r > 0.0 and m.range_min <= r <= m.range_max)
        self.valid = v / n

    def set_mode(self, main, sub):
        v = VehicleCommand()
        v.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        v.param1, v.param2, v.param3 = 1.0, float(main), float(sub)
        v.target_system = v.source_system = 1
        v.target_component = v.source_component = 1
        v.from_external = True
        v.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vcmd.publish(v)

    def drive(self, vx):
        t = Twist()
        t.linear.x = vx
        self.cmd.publish(t)


def spin(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--duration', type=float, default=70.0)
    a = ap.parse_args()

    rclpy.init()
    n = S2Stands(a.speed)

    # DDS discovery time is variable - a fixed 2 s wait aborted a good run on
    # 2026-08-10 while the topic was perfectly healthy. Wait for the data.
    deadline = time.time() + 12.0
    while time.time() < deadline and n.arming is None:
        rclpy.spin_once(n, timeout_sec=0.2)

    if n.arming is None:
        print('ABORT: no vehicle_status after 12 s - is microxrce-agent up?')
        return 2
    if n.arming == ARM_ARMED:
        print('ABORT: vehicle is ARMED. This test is disarmed-only.')
        return 2
    print(f'disarmed (arming_state={n.arming}), nav_state={n.nav}, '
          f'scan validity={100*(n.valid or 0):.1f}%')

    # Engage AutoNav, then verify it actually took before commanding anything.
    print('engaging AutoNav (disarmed)...')
    for _ in range(5):
        n.set_mode(MAIN_CUSTOM, SUB_AUTONAV)
        spin(n, 0.4)
    if n.nav != NAV_AUTONAV:
        print(f'ABORT: AutoNav did not engage (nav_state={n.nav}, expected {NAV_AUTONAV})')
        return 2
    print(f'AutoNav ACTIVE (nav_state={n.nav}). Commanding {a.speed:.2f} m/s forward.')

    # INSTRUMENT CHECK. A subscription that receives nothing is indistinguishable
    # from a reflex that never fires, so prove the channel works before the run.
    for _ in range(60):
        n.drive(a.speed)
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.sp_n > 0:
            break
    if n.sp_n == 0:
        print('ABORT: no messages on /fmu/in/rover_speed_setpoint after 3 s.')
        print('       The instrument is broken - do NOT read this as "reflex did not fire".')
        n.drive(0.0)
        return 2
    print(f'instrument OK: {n.sp_n} setpoint msgs, last speed_body_x={n.sp:.3f}\n')

    print(f"{'t(s)':>6} {'valid%':>7} {'cmd':>6} {'setpoint':>9}  verdict")
    print('-' * 52)
    t0 = time.time()
    nxt = 0.0
    blocked_samples = passed_samples = 0
    while True:
        n.drive(a.speed)
        rclpy.spin_once(n, timeout_sec=0.02)
        el = time.time() - t0

        if n.arming == ARM_ARMED:
            print('\nABORT: vehicle became ARMED. Stopping and zeroing.')
            break
        if n.nav != NAV_AUTONAV:
            print(f'\nABORT: AutoNav dropped out (nav_state={n.nav}). Stopping.')
            break

        if el >= nxt:
            v = 100 * (n.valid if n.valid is not None else 0)
            sp = n.sp
            if sp is None:
                verd = 'NO SETPOINT SEEN'
            elif sp <= 0.001:
                verd = 'BLOCKED (speed forced to 0)'
                blocked_samples += 1
            else:
                verd = 'passing through'
                passed_samples += 1
            sps = '--' if sp is None else f'{sp:.3f}'
            print(f'{el:6.1f} {v:7.1f} {a.speed:6.2f} {sps:>9}  {verd}')
            nxt += 1.0

        if el >= a.duration:
            break

    n.drive(0.0)
    spin(n, 0.5)
    n.drive(0.0)
    print(f'\ncommanded zero. setpoint msgs seen: {n.sp_n}')
    print(f'samples blocked={blocked_samples} passing={passed_samples}')
    print('AutoNav left engaged and disarmed; it holds zero with no /cmd_vel.')
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
