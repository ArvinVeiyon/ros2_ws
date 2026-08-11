#!/usr/bin/env python3
"""Collision standoff — does the rover actually stop >=300 mm short of a WALL?

This is NOT S2. S2 kills /scan and asks whether a blind rover stops; it is run
with a deliberately CLEAR corridor so no obstacle can confound the timing. This
test is the opposite: /scan stays healthy throughout, and the rover drives at a
real obstacle until the reflex fires. The question here is DISTANCE, not latency.

REQUIREMENT UNDER TEST
  collision.stop_distance = 0.35 m at the bumper, which is required to leave
  >=300 mm of standoff. The 2026-07-22 armed wall test stopped short of a wall,
  but under the older geometry, before stop_distance was corrected to 0.35 at
  the bumper - so it does not certify the current parameters.

WHY THIS IS NOT ALREADY IMPLIED BY S2
  S2 measured the reflex COASTING to a stop: ~0.49-0.57 s from setpoint-zero to
  rpm-zero, mechanical and variable (2026-08-11, n=2). Against a wall there is
  no 500 ms scan timeout - the reflex fires within about one update tick - so
  the standoff is set by the trigger distance MINUS whatever the coast eats.
  Arithmetic from the measured coast predicts ~0.33 m at 0.08 m/s, ~0.28 m at
  0.30 m/s. That is a PREDICTION from a coast time assumed constant with speed,
  which is exactly the assumption nobody has measured. Hence: measure it.

WHAT IT RECORDS, and why /odom is here
  Every distance quoted after the S2 runs was arithmetic from the COMMANDED
  speed, because those runs never logged /odom. For a safety number that is not
  good enough. This logs /odom (both integrated speed and raw pose displacement)
  so travel is observed, and reports the front clearance from /scan at three
  instants: reflex fire, wheels stopped, and settled.

THIS MOVES A LIVE VEHICLE AT A WALL. Hand ON the kill switch (ch8). FLOOR only.
Never arms, never disarms. Requires rover-ekf-bridge running (setup_manual C10).

  python3 tools/collision_standoff_test.py                 # 0.08 m/s
  python3 tools/collision_standoff_test.py --speed 0.12 --max-travel 1.2
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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import EscStatus, RoverSpeedSetpoint, VehicleCommand, VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
# /fmu/in/* are published by px4_ros2 INSIDE the mode and are VOLATILE, unlike
# the /fmu/out/* the DDS agent publishes TRANSIENT_LOCAL. Subscribing
# TRANSIENT_LOCAL here receives NOTHING while looking like a quiet topic.
SETPOINT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)

ARM_ARMED, NAV_AUTONAV = 2, 23
MAIN_CUSTOM, SUB_AUTONAV = 4, 11

# Must match the mode's declared parameters (autonav_mode/mode.hpp). If you change
# them there, change them here - a mismatch silently reports the wrong geometry.
FRONT_OVERHANG = 0.337    # [m] bumper is this far ahead of the camera
CORRIDOR_HALF = 0.28      # [m] half width of the swept corridor
SECTOR_HALF = math.radians(20.0)
STOP_DISTANCE = 0.35      # [m] bumper clearance at which forward is blocked
REQUIRED_STANDOFF = 0.30  # [m] the requirement this test exists to check


class Standoff(Node):
    def __init__(self):
        super().__init__('collision_standoff')
        self.arming = self.nav = None
        self.rpm = {}
        self.bumper = None        # front clearance at the bumper [m]
        self.valid = None         # whole-scan valid fraction
        self.scan_t = None
        self.sp = None
        self.sp_n = 0
        self.odom_x = self.odom_y = None
        self.odom_x0 = self.odom_y0 = None
        self.odom_vx = 0.0

        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.st_cb, PX4_QOS)
        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc_cb, PX4_QOS)
        self.create_subscription(RoverSpeedSetpoint, '/fmu/in/rover_speed_setpoint',
                                 self.sp_cb, SETPOINT_QOS)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vcmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

    def st_cb(self, m):
        self.arming, self.nav = m.arming_state, m.nav_state

    def esc_cb(self, m):
        self.rpm = {e.esc_address: e.esc_rpm for e in m.esc[:m.esc_count]}

    def sp_cb(self, m):
        self.sp = m.speed_body_x
        self.sp_n += 1

    def odom_cb(self, m):
        self.odom_x = m.pose.pose.position.x
        self.odom_y = m.pose.pose.position.y
        self.odom_vx = m.twist.twist.linear.x
        if self.odom_x0 is None:
            self.odom_x0, self.odom_y0 = self.odom_x, self.odom_y

    def scan_cb(self, m):
        # Same corridor geometry the reflex uses: resolve each ray into forward
        # (x) and lateral (y) and keep only what the BODY will sweep. A cone
        # narrows as it approaches the sensor and would miss an obstacle sitting
        # inside the body width - see mode.hpp for why this shape, not a sector.
        best = float('inf')
        valid = 0
        n = len(m.ranges)
        for i, r in enumerate(m.ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0.0:
                continue
            valid += 1
            a = m.angle_min + i * m.angle_increment
            if abs(a) > SECTOR_HALF:
                continue
            x, y = r * math.cos(a), r * math.sin(a)
            if x > 0.0 and abs(y) <= CORRIDOR_HALF:
                best = min(best, x)
        self.bumper = (best - FRONT_OVERHANG) if math.isfinite(best) else float('inf')
        self.valid = valid / n if n else 0.0
        self.scan_t = time.time()

    def max_rpm(self):
        return max((abs(v) for v in self.rpm.values()), default=0)

    def travel(self):
        if self.odom_x is None or self.odom_x0 is None:
            return float('nan')
        return math.hypot(self.odom_x - self.odom_x0, self.odom_y - self.odom_y0)

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
        t = Twist()
        t.linear.x = float(vx)
        self.cmd.publish(t)

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)


def fail(n, msg, code=2):
    try:
        n.drive(0.0)
        n.spin(0.4)
        n.drive(0.0)
    finally:
        print(msg)
        rclpy.shutdown()
        sys.exit(code)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.08)
    ap.add_argument('--max-travel', type=float, default=1.0,
                    help='hard stop: command zero after this much odom travel')
    ap.add_argument('--bound', type=float, default=40.0, help='hard time limit [s]')
    ap.add_argument('--min-approach', type=float, default=0.60,
                    help='refuse to start unless the wall is at least this far past stop_distance')
    ap.add_argument('--spinup', type=float, default=4.0,
                    help='abort if the wheels have not turned by this point [s]')
    a = ap.parse_args()

    rclpy.init()
    n = Standoff()
    n.spin(6.0)

    if n.arming is None:
        fail(n, 'no vehicle_status - is the DDS agent up?')
    if n.scan_t is None:
        fail(n, 'no /scan - nothing to see the wall with')
    if n.odom_x is None:
        fail(n, 'no /odom - travel could not be measured, which is the point of this test')
    if n.arming != ARM_ARMED:
        fail(n, 'NOT ARMED. Arm via RC first; this script never arms.', 1)
    if n.max_rpm() != 0:
        fail(n, f'WHEELS ALREADY TURNING (rpm {n.max_rpm()}). Aborting.', 1)
    if n.valid is not None and n.valid < 0.35:
        fail(n, f'scan validity {100*n.valid:.1f}% is below the mode gate (35%) - it would block blind.')

    b0 = n.bumper
    print(f'state: arming={n.arming} nav={n.nav} rpm=0 validity={100*n.valid:.1f}%')
    print(f'wall at bumper clearance {b0:.2f} m  (stop_distance {STOP_DISTANCE:.2f} m)')

    # An obstacle must actually BE there, and far enough that the run measures a
    # stop rather than starting inside the trigger. Starting already-blocked
    # produces "wheels never turned", which says nothing about standoff.
    if not math.isfinite(b0):
        fail(n, 'NO OBSTACLE in the corridor - this test needs a wall to drive at.')
    if b0 < STOP_DISTANCE + a.min_approach:
        fail(n, f'wall too close to start ({b0:.2f} m). Need >= '
                f'{STOP_DISTANCE + a.min_approach:.2f} m, or lower --min-approach deliberately.')

    print('\nswitching to AutoNav ...')
    for _ in range(5):
        n.set_mode(MAIN_CUSTOM, SUB_AUTONAV)
        n.spin(0.4)
    n.spin(2.0)
    if n.nav != NAV_AUTONAV:
        fail(n, f'AutoNav did not hold (nav={n.nav}). No motion. Aborting.', 1)
    print(f'AutoNav holding (nav={n.nav}).')

    print(f'\n>>> DRIVING AT THE WALL at {a.speed} m/s <<<')
    print(f'    hand on ch8. hard limits: {a.max_travel:.2f} m travel, {a.bound:.0f} s\n')
    print(f"{'t(s)':>6} {'bumper':>7} {'travel':>7} {'odom v':>7} {'setpt':>6} {'rpm':>5}")
    print('-' * 46)

    t0 = time.time()
    nxt = 0.0
    fired_at = None       # wall time the emitted setpoint first went to zero
    b_fire = None         # bumper clearance at that instant
    stopped_at = None
    b_stop = None
    reason = 'bound'

    while True:
        n.drive(a.speed)
        rclpy.spin_once(n, timeout_sec=0.02)
        el = time.time() - t0

        if n.arming != ARM_ARMED:
            reason = 'DISARMED mid-run'
            break
        if n.nav != NAV_AUTONAV:
            reason = f'AutoNav dropped (nav={n.nav})'
            break

        # Below the friction deadband the rover simply never moves, and every
        # instant below reads as "the reflex has not fired yet" - so the run
        # would sit still for the full bound and report nothing. At reduced
        # speed this is the LIKELY outcome, not an edge case. Catch it early and
        # say so, rather than producing a silent non-result.
        if el >= a.spinup and n.max_rpm() == 0 and fired_at is None:
            n.drive(0.0)
            n.spin(0.5)
            print(f'\nWHEELS NEVER TURNED in {a.spinup:.1f}s at {a.speed:.3f} m/s '
                  f'(travel {n.travel():.3f} m).')
            print('  Below the friction deadband, or the reflex is already blocking.')
            print(f'  bumper clearance {n.bumper:.2f} m, stop_distance {STOP_DISTANCE:.2f} m')
            print('  INCONCLUSIVE - not a standoff measurement. Try a higher --speed.')
            rclpy.shutdown()
            sys.exit(3)

        # The reflex firing is read from what the mode EMITS, not from a log line.
        if fired_at is None and n.sp is not None and abs(n.sp) <= 0.001 and n.max_rpm() > 0:
            fired_at, b_fire = time.time(), n.bumper
        if fired_at is not None and stopped_at is None and n.max_rpm() == 0:
            stopped_at, b_stop = time.time(), n.bumper
            reason = 'stopped'
            break

        if n.travel() >= a.max_travel:
            reason = f'HARD TRAVEL LIMIT {a.max_travel:.2f} m'
            break
        if el >= a.bound:
            break

        if el >= nxt:
            bs = 'inf' if not math.isfinite(n.bumper) else f'{n.bumper:.2f}'
            print(f'{el:6.1f} {bs:>7} {n.travel():7.3f} {n.odom_vx:7.3f} '
                  f'{(n.sp if n.sp is not None else float("nan")):6.3f} {n.max_rpm():5d}')
            nxt += 0.5

    n.drive(0.0)
    n.spin(1.5)                       # let it settle before reading the final clearance
    n.drive(0.0)
    b_settled = n.bumper
    travel = n.travel()

    print('\n============== STANDOFF RESULT ==============')
    print(f'  stop reason                 {reason}')
    print(f'  commanded speed             {a.speed:.3f} m/s')
    print(f'  bumper clearance at start   {b0:.3f} m')
    if fired_at is not None:
        print(f'  bumper clearance at FIRE    {b_fire:.3f} m   '
              f'(stop_distance {STOP_DISTANCE:.2f} m)')
    else:
        print('  reflex NEVER FIRED')
    if b_stop is not None:
        print(f'  bumper clearance at STOP    {b_stop:.3f} m')
        print(f'  coast distance              {b_fire - b_stop:.3f} m')
    print(f'  bumper clearance SETTLED    {b_settled:.3f} m   <== the standoff')
    print(f'  odom travel                 {travel:.3f} m')
    print('')

    if b_settled >= REQUIRED_STANDOFF:
        print(f'  ✅ PASSES: {b_settled:.3f} m >= {REQUIRED_STANDOFF:.2f} m required.')
    else:
        print(f'  🔴 FAILS: {b_settled:.3f} m < {REQUIRED_STANDOFF:.2f} m required.')
    print('  NOTE: this standoff is measured by /scan. Tape-measure the gap and')
    print('        record BOTH - they answer different questions, and a')
    print('        disagreement is itself a finding about the depth sensor.')
    print('=============================================')
    print(f'final: arming={n.arming} nav={n.nav} rpm={n.max_rpm()}')
    print('AutoNav left engaged; it holds zero with no /cmd_vel. /scan untouched.')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
