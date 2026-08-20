#!/usr/bin/env python3
"""T2 — straight goal down a CLEAR corridor. The first autonomous drive.

REQUIREMENT UNDER TEST (autonav_reference.md section 12)
  "Straight goal, clear 2 m corridor. Arrives within 0.20 m, no collision stop."

  Two criteria, and they fail in opposite directions. Arriving needs the rover to
  KEEP GOING; "no collision stop" needs the reflex to stay silent on a corridor
  that has nothing in it. A run where the reflex fires is a FAIL even if the
  rover ends up in the right place, because it means the reflex is firing on a
  clear corridor.

WHY THIS IS NOT THE STANDOFF TEST WITH A DIFFERENT NUMBER
  collision_standoff_test.py drives AT a wall and measures where the reflex
  stops it. This drives at NOTHING and asks whether the vehicle can hold a
  straight line to a commanded distance without the reflex intervening. The
  standoff test wants an obstacle and fails without one; this one wants a clear
  corridor and fails WITH one.

>>> THE PASS/FAIL IS ADJUDICATED ON TAPE, NOT ON /odom. <<<
  The criterion is 0.20 m over 2 m. /odom under-reads by ~24% at crawl
  (2026-08-13, ESC zero-dropout - the scale 0.003900 is NOT the defect and must
  not be "corrected"). 24% of 2 m is 0.48 m, so the instrument's known error is
  more than twice the tolerance it would be adjudicating. A run that reports
  "arrived within 0.20 m by /odom" would be measuring nothing.

  So: --distance is denominated in ODOM METRES and is only how the run TERMINATES.
  Whether T2 passed is decided by --tape-start/--tape-end in a second --analyse
  pass. Same method that settled the standoff (n=3, taped 0.345 m) and the map
  scale. Without tape this tool returns INCONCLUSIVE, by design.

>>> SIZE THE CORRIDOR FOR THE UNDER-READ, NOT FOR --distance. <<<
  Because /odom under-reads, the rover travels FURTHER than the odom goal - a
  2.0 m odom goal is roughly 2.6 m of real ground. The corridor must clear the
  worst case, not the nominal. This tool refuses to start unless the measured
  clearance covers distance * ODOM_WORST_CASE + stop_distance, and the
  --min-bumper backstop (which does not depend on odom scale) guards the rest.

THIS MOVES A LIVE VEHICLE. Hand ON the kill switch (ch8). FLOOR only.
Never arms, never disarms. Requires rover-ekf-bridge running (setup_manual C10).
GATE ON MEASURED SPEED, NEVER ON THE COMMAND (2026-08-12: commanded 0.25 gave
~0.9 m/s and the rover hit a wall).

  python3 tools/t2_straight_goal_test.py                       # 2 m at 0.08 m/s
  python3 tools/t2_straight_goal_test.py --distance 1.5
  python3 tools/t2_straight_goal_test.py --analyse t2_*.json \
      --tape-start 2.40 --tape-end 0.35
"""
import argparse
import json
import math
import os
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

GOAL_TOLERANCE = 0.20     # [m] the T2 requirement, adjudicated on TAPE
# /odom under-reads at crawl (~24%, 2026-08-13). Real ground distance is
# therefore LONGER than the odom reading. Size the corridor for the worst case.
ODOM_WORST_CASE = 1.35


class T2(Node):
    def __init__(self):
        super().__init__('t2_straight_goal')
        self.arming = self.nav = None
        self.rpm = {}
        self.bumper = None        # front clearance at the bumper [m]
        self.valid = None         # whole-scan valid fraction
        self.scan_t = None
        self.sp = None
        self.sp_n = 0
        self.odom_x = self.odom_y = None
        self.odom_x0 = self.odom_y0 = None
        self.odom_yaw0 = None
        self.odom_vx = 0.0
        self.yaw = None
        self.esc_log = []         # (t, left mean |erpm|, right mean |erpm|)
        self.track_log = []       # (t, travel, lateral offset, yaw error)

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
        l = [abs(self.rpm[a]) for a in (10, 11) if a in self.rpm]
        r = [abs(self.rpm[a]) for a in (12, 13) if a in self.rpm]
        if l and r:
            self.esc_log.append((time.time(), sum(l)/len(l), sum(r)/len(r)))

    def sp_cb(self, m):
        self.sp = m.speed_body_x
        self.sp_n += 1

    def odom_cb(self, m):
        self.odom_x = m.pose.pose.position.x
        self.odom_y = m.pose.pose.position.y
        self.odom_vx = m.twist.twist.linear.x
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.odom_x0 is None:
            self.odom_x0, self.odom_y0 = self.odom_x, self.odom_y
            self.odom_yaw0 = self.yaw

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

    def lateral(self):
        """Signed offset from the straight line the rover started pointing along.

        T2 is a STRAIGHT goal - drifting 0.5 m sideways while covering 2 m
        forward is not an arrival. Reported for diagnosis only: this is /odom
        heading, which is the camera gyro, so it is subject to the same doubts
        as everything else derived from it. The tape is what adjudicates.
        """
        if self.odom_x0 is None or self.odom_yaw0 is None:
            return float('nan')
        dx, dy = self.odom_x - self.odom_x0, self.odom_y - self.odom_y0
        return -dx * math.sin(self.odom_yaw0) + dy * math.cos(self.odom_yaw0)

    def yaw_err(self):
        if self.yaw is None or self.odom_yaw0 is None:
            return float('nan')
        return math.degrees(math.atan2(math.sin(self.yaw - self.odom_yaw0),
                                       math.cos(self.yaw - self.odom_yaw0)))

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


def analyse(path, tape_start, tape_end):
    """Adjudicate T2 on the tape, and report what /odom claimed for comparison.

    tape_start / tape_end are bumper-to-END-WALL clearances measured BEFORE the
    run and AFTER the rover settled. Their difference is real ground distance
    travelled, independent of odom scale and of the ESC dropout.
    """
    with open(path) as f:
        d = json.load(f)

    real = tape_start - tape_end
    goal = d['distance']
    odom = d['travel_odom']
    err = real - goal
    ratio = (odom / real) if real > 0 else float('nan')

    print(f'\nT2 run {os.path.basename(path)}')
    print(f'  commanded speed      {d["speed"]:.3f} m/s')
    print(f'  odom goal            {goal:.3f} m   (termination criterion only)')
    print(f'  /odom travel         {odom:.3f} m')
    print(f'  TAPED real distance  {real:.3f} m   ({tape_start:.3f} -> {tape_end:.3f})')
    print(f'  error vs goal        {err:+.3f} m   (tolerance +/-{GOAL_TOLERANCE:.2f} m)')
    print(f'  odom/real ratio      {ratio:.3f}    (1.000 = odom truthful)')
    if math.isfinite(d.get('lateral', float('nan'))):
        print(f'  lateral drift        {d["lateral"]:+.3f} m  (odom-derived, diagnostic)')
    print(f'  reflex fired         {"YES" if d["reflex_fired"] else "no"}')
    print(f'  terminated because   {d["reason"]}')

    arrived = abs(err) <= GOAL_TOLERANCE
    clean = not d['reflex_fired']
    print()
    if arrived and clean:
        print(f'  ✅ T2 PASS - arrived within {GOAL_TOLERANCE:.2f} m, no collision stop.')
    else:
        if not arrived:
            print(f'  ❌ T2 FAIL - {abs(err):.3f} m off the goal, tolerance is '
                  f'{GOAL_TOLERANCE:.2f} m.')
        if not clean:
            print('  ❌ T2 FAIL - the reflex fired on a CLEAR corridor. That is a '
                  'false positive,')
            print('     and it matters more than the distance: investigate before rerunning.')
    print(f'\n  NOTE: the odom/real ratio above is a free by-product, not a licence to '
          f'\n  retune erpm_to_ms. That constant is CLOSED at 0.003900 (2026-08-13); the '
          f'\n  discrepancy is the ESC zero-dropout. One run cannot separate them.')
    return 0 if (arrived and clean) else 1


def fail(n, msg, code=2):
    n.drive(0.0)
    n.spin(0.3)
    print(f'\n⛔ {msg}')
    rclpy.shutdown()
    sys.exit(code)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--distance', type=float, default=2.0,
                    help='ODOM metres at which the run terminates (NOT the pass criterion)')
    ap.add_argument('--speed', type=float, default=0.08,
                    help='commanded m/s - crawl only, speed increase is not permitted')
    ap.add_argument('--bound', type=float, default=90.0, help='hard time limit [s]')
    ap.add_argument('--spinup', type=float, default=4.0,
                    help='abort if the wheels have not turned by this point [s]')
    ap.add_argument('--max-measured-speed', type=float, default=0.20,
                    help='ABORT if /odom shows the rover exceeding this [m/s]')
    ap.add_argument('--min-bumper', type=float, default=0.15,
                    help='ABORT if clearance falls below this [m] - the odom-independent backstop')
    ap.add_argument('--analyse', help='JSON from a previous run')
    ap.add_argument('--tape-start', type=float, help='taped bumper->end wall BEFORE the run')
    ap.add_argument('--tape-end', type=float, help='taped bumper->end wall AFTER it settled')
    a = ap.parse_args()

    if a.analyse:
        if a.tape_start is None or a.tape_end is None:
            print('--analyse needs --tape-start and --tape-end.')
            print('Without tape this run is INCONCLUSIVE - /odom cannot adjudicate a '
                  '0.20 m tolerance when it under-reads by ~24%.')
            return 2
        return analyse(a.analyse, a.tape_start, a.tape_end)

    rclpy.init()
    n = T2()
    n.spin(6.0)

    if n.arming is None:
        fail(n, 'no vehicle_status - is the DDS agent up?')
    if n.scan_t is None:
        fail(n, 'no /scan - the reflex would be blind and this test needs it watching')
    if n.odom_x is None:
        fail(n, 'no /odom - the run could not be terminated on distance')
    if n.arming != ARM_ARMED:
        fail(n, 'NOT ARMED. Arm via RC first; this script never arms.', 1)
    if n.max_rpm() != 0:
        fail(n, f'WHEELS ALREADY TURNING (rpm {n.max_rpm()}). Aborting.', 1)
    if n.valid is not None and n.valid < 0.35:
        fail(n, f'scan validity {100*n.valid:.1f}% is below the mode gate (35%) - it '
                f'would block blind, and a blind block is not a T2 result.')

    # THE CORRIDOR MUST BE CLEAR, and clear for the WORST-CASE real distance -
    # not for --distance, which is denominated in under-reading odom metres.
    b0 = n.bumper
    needed = a.distance * ODOM_WORST_CASE + STOP_DISTANCE
    print(f'state: arming={n.arming} nav={n.nav} rpm=0 validity={100*n.valid:.1f}%')
    bs0 = 'inf (nothing in the corridor)' if not math.isfinite(b0) else f'{b0:.2f} m'
    print(f'corridor clearance ahead: {bs0}')
    print(f'need >= {needed:.2f} m  ({a.distance:.2f} odom m x {ODOM_WORST_CASE:.2f} '
          f'worst-case under-read + {STOP_DISTANCE:.2f} stop distance)')
    if math.isfinite(b0) and b0 < needed:
        fail(n, f'CORRIDOR TOO SHORT ({b0:.2f} m). The rover travels further than '
                f'--distance because /odom under-reads;\n   at 2 m odom that is ~2.6 m of '
                f'real ground. Lengthen the corridor or lower --distance.')

    print('\nswitching to AutoNav ...')
    for _ in range(5):
        n.set_mode(MAIN_CUSTOM, SUB_AUTONAV)
        n.spin(0.4)
    n.spin(2.0)
    if n.nav != NAV_AUTONAV:
        fail(n, f'AutoNav did not hold (nav={n.nav}). No motion. Aborting.', 1)
    print(f'AutoNav holding (nav={n.nav}).')

    print(f'\n>>> DRIVING {a.distance:.2f} ODOM m DOWN A CLEAR CORRIDOR at {a.speed} m/s <<<')
    print(f'    hand on ch8. backstop: {a.min_bumper:.2f} m clearance, {a.bound:.0f} s\n')
    print(f"{'t(s)':>6} {'travel':>7} {'lat':>7} {'yaw':>7} {'odom v':>7} "
          f"{'bumper':>7} {'setpt':>6} {'rpm':>5}")
    print('-' * 62)

    n.esc_log.clear()
    t0 = time.time()
    nxt = 0.0
    reflex_fired = False
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

        # Below the friction deadband the rover never moves and the run would sit
        # still for the full bound reporting nothing. At crawl this is the LIKELY
        # outcome, not an edge case.
        if el >= a.spinup and n.max_rpm() == 0 and not reflex_fired:
            n.drive(0.0)
            n.spin(0.5)
            print(f'\nWHEELS NEVER TURNED in {a.spinup:.1f}s at {a.speed:.3f} m/s.')
            print('  Below the friction deadband. INCONCLUSIVE - not a T2 result.')
            rclpy.shutdown()
            sys.exit(3)

        # The reflex firing on a CLEAR corridor is a T2 failure, not a safety
        # event - but it ends the run either way, because everything after it is
        # a measurement of the reflex rather than of the goal.
        if not reflex_fired and n.sp is not None and abs(n.sp) <= 0.001 and n.max_rpm() > 0:
            reflex_fired = True
            reason = 'REFLEX FIRED on a clear corridor'
            print(f'\n⚠ reflex fired at travel {n.travel():.3f} m, '
                  f'bumper {n.bumper:.2f} m - this is a T2 FAIL')
            break

        # GATE ON MEASURED SPEED, NEVER ON THE COMMAND.
        # 2026-08-12: commanded 0.25 m/s produced ~0.9 m/s actual (3.6x) and the
        # rover hit the wall. The command is not a speed - only /odom knows how
        # fast this vehicle is going, and even it under-reads, so this trips LATE.
        if abs(n.odom_vx) > a.max_measured_speed:
            n.drive(0.0); n.spin(0.6); n.drive(0.0)
            print(f'\n⛔ ABORT: measured {abs(n.odom_vx):.2f} m/s exceeds the '
                  f'{a.max_measured_speed:.2f} m/s limit (commanded {a.speed:.2f}).')
            reason = 'MEASURED SPEED LIMIT'
            break
        # ODOM-INDEPENDENT BACKSTOP. Clearance does not depend on odom scale, so
        # this is the guard that still works when the odometry is lying - which,
        # at crawl, it is known to be.
        if math.isfinite(n.bumper) and n.bumper < a.min_bumper:
            n.drive(0.0); n.spin(0.6); n.drive(0.0)
            print(f'\n⛔ ABORT: clearance {n.bumper:.3f} m fell below the '
                  f'{a.min_bumper:.2f} m backstop.')
            reason = 'MIN BUMPER BACKSTOP'
            break
        if n.travel() >= a.distance:
            reason = 'reached odom goal'
            break
        if el >= a.bound:
            break

        if el >= nxt:
            bs = 'inf' if not math.isfinite(n.bumper) else f'{n.bumper:.2f}'
            print(f'{el:6.1f} {n.travel():7.3f} {n.lateral():+7.3f} {n.yaw_err():+7.2f} '
                  f'{n.odom_vx:7.3f} {bs:>7} '
                  f'{(n.sp if n.sp is not None else float("nan")):6.3f} {n.max_rpm():5d}')
            nxt += 0.5
            n.track_log.append((el, n.travel(), n.lateral(), n.yaw_err()))

    n.drive(0.0)
    n.spin(1.5)          # let it settle before the final reading
    n.drive(0.0)

    out = {
        'distance': a.distance,
        'speed': a.speed,
        'travel_odom': n.travel(),
        'lateral': n.lateral(),
        'yaw_err_deg': n.yaw_err(),
        'bumper_final': n.bumper if math.isfinite(n.bumper) else None,
        'reflex_fired': reflex_fired,
        'reason': reason,
        'esc_log': n.esc_log,
        'track_log': n.track_log,
    }
    path = f't2_{time.strftime("%Y%m%d_%H%M%S")}.json'
    with open(path, 'w') as f:
        json.dump(out, f)

    print(f'\nstopped: {reason}')
    print(f'  /odom travel   {n.travel():.3f} m')
    print(f'  lateral drift  {n.lateral():+.3f} m   yaw {n.yaw_err():+.2f} deg')
    print(f'  reflex fired   {"YES - T2 FAIL" if reflex_fired else "no"}')
    print(f'  saved          {path}')
    print('\n>>> NOW TAPE bumper -> end wall, then:')
    print(f'    python3 tools/t2_straight_goal_test.py --analyse {path} \\')
    print(f'        --tape-start <before> --tape-end <after>')
    print('    Until you do, this run is INCONCLUSIVE - /odom cannot adjudicate a')
    print('    0.20 m tolerance when its own error at crawl is ~0.48 m over 2 m.')

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
