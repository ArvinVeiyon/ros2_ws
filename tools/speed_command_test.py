#!/usr/bin/env python3
"""Speed commands are not honoured — WHICH half of the controller is at fault?

THE FAULT
  Commanded 0.05 -> ~0.11 m/s, 0.06 -> 0.146, 0.25 -> ~0.9 (3.6x, and the rover
  contacted a wall on 2026-08-12). The ratio GROWS with the command, which a
  plain wrong scale factor cannot produce. Until this is understood every
  standoff guarantee is conditional on the vehicle happening to crawl.

THE CONTROLLER, from the firmware actually flashed (a52c38b07d)
  DifferentialSpeedControl -> RoverControl::speedControl (RoverControl.cpp:107):

      throttle = setpoint / RO_MAX_THR_SPEED        <- feedforward
               + PID(setpoint - vehicle_speed)      <- feedback, clamped to 1.0

  RO_MAX_THR_SPEED = 0.60, RO_SPEED_P = 0.5, RO_SPEED_I = 0.1 (integral limit
  1.0). `vehicle_speed` is taken from vehicle_local_position.vx/vy at
  DifferentialSpeedControl.cpp:100-107 -- and v_xy_valid is NEVER CHECKED.
  Line 106 also deadbands it: anything under RO_SPEED_TH (0.10 m/s) is forced to
  EXACTLY ZERO, so at crawl the feedback is zero by construction.

TWO HYPOTHESES, and they predict different logs
  H1 INTEGRAL WINDUP. The EKF velocity is dead (measured 2026-08-10 and again on
     this boot: v_xy_valid false, dead_reckoning true), so the PID believes the
     rover is stationary, the integral ramps at RO_SPEED_I * error and throttle
     climbs to saturation.
     PREDICTS: throttle STARTS near the feedforward value and CLIMBS roughly
     linearly at ~0.1*error per second; EKF speed stays under 0.10 m/s while
     /odom shows real motion.
  H2 FEEDFORWARD CONSTANT WRONG. RO_MAX_THR_SPEED = 0.60 understates the true
     full-throttle speed, so every command is scaled up by a fixed ratio.
     PREDICTS: throttle sits FLAT at setpoint/0.60 and does not climb; the
     overspeed ratio is the same at every command.
  They are not exclusive -- both can be true, and the log separates them because
  one is a RAMP and the other is an OFFSET.

WHAT IT RECORDS
  /fmu/out/rover_throttle_setpoint  the controller's own output (the answer)
  /fmu/out/vehicle_local_position_v1 the feedback it is closing the loop on
  /odom                             what the wheels say actually happened
  /fmu/in/rover_speed_setpoint      what the mode emitted, post-reflex

  rover_speed_status -- which carries the PID integral directly -- is NOT in
  dds_topics.yaml, so throttle saturation is the proxy for windup. Adding it is
  a firmware flash; pair it with the vehicle_angular_velocity item if that ever
  happens.

THIS MOVES A LIVE VEHICLE. Hand ON the kill switch (ch8). FLOOR only, and it
wants a CLEAR corridor -- unlike the standoff test there is no wall to stop it,
so the gates below are the only thing that does. Never arms, never disarms.
Requires rover-ekf-bridge running (setup_manual C10) -- without it the EKF
velocity is dead for a DIFFERENT reason and the run measures nothing.

  python3 tools/speed_command_test.py                    # 0.05 m/s, 20 s
  python3 tools/speed_command_test.py --speed 0.10 --max-measured-speed 0.30
  python3 tools/speed_command_test.py --analyse run.json
"""
import argparse
import json
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import (EscStatus, RoverSpeedSetpoint, RoverThrottleSetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

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

# Geometry, matching autonav_mode/mode.hpp. Used only for the clearance backstop.
FRONT_OVERHANG = 0.337
CORRIDOR_HALF = 0.28
SECTOR_HALF = math.radians(20.0)

# PX4 params, read live on 2026-08-12. These are the numbers the predictions are
# built from -- if they are changed on the FC, change them here or the run will
# be scored against the wrong feedforward.
RO_MAX_THR_SPEED = 0.60
RO_SPEED_P = 0.5
RO_SPEED_I = 0.1
RO_SPEED_TH = 0.10       # feedback deadband: |v| below this reads as EXACTLY 0

# Below this mean throttle, speed is NOT proportional to throttle (stiction,
# stick-slip, ESC commutation floor) and speed/throttle stops estimating the
# full-throttle speed. See the 2026-08-12 run: mean throttle 0.032 implied
# 4.32 m/s on a ~0.6 m/s vehicle.
THROTTLE_FLOOR_FOR_IMPLIED = 0.15


class SpeedProbe(Node):
    def __init__(self):
        super().__init__('speed_command_test')
        self.arming = self.nav = None
        self.rpm = {}
        self.bumper = None
        self.valid = None
        self.scan_t = None
        self.sp = None                    # emitted speed setpoint [m/s]
        self.thr = None                   # controller throttle output [-1,1]
        self.thr_n = 0
        self.ekf_v = None                 # |EKF horizontal velocity| [m/s]
        self.ekf_valid = None
        self.ekf_dr = None
        self.odom_x = self.odom_y = None
        self.odom_x0 = self.odom_y0 = None
        self.odom_vx = 0.0
        self.samples = []                 # the log this test exists to produce

        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.st_cb, PX4_QOS)
        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc_cb, PX4_QOS)
        self.create_subscription(RoverThrottleSetpoint, '/fmu/out/rover_throttle_setpoint',
                                 self.thr_cb, PX4_QOS)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 self.lp_cb, PX4_QOS)
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

    def thr_cb(self, m):
        self.thr = m.throttle_body_x
        self.thr_n += 1

    def lp_cb(self, m):
        # The controller rotates NED velocity into body and takes the norm, then
        # deadbands it. The norm alone decides whether the feedback is zero, and
        # it needs no attitude -- which keeps this independent of the heading
        # that is separately known to be unusable.
        self.ekf_v = math.hypot(m.vx, m.vy)
        self.ekf_valid = m.v_xy_valid
        self.ekf_dr = m.dead_reckoning

    def sp_cb(self, m):
        self.sp = m.speed_body_x

    def odom_cb(self, m):
        self.odom_x = m.pose.pose.position.x
        self.odom_y = m.pose.pose.position.y
        self.odom_vx = m.twist.twist.linear.x
        if self.odom_x0 is None:
            self.odom_x0, self.odom_y0 = self.odom_x, self.odom_y

    def scan_cb(self, m):
        # Same corridor shape the reflex uses (mode.hpp): resolve each ray into
        # forward/lateral and keep only what the BODY will sweep. Here it is
        # only the abort backstop -- this test wants an EMPTY corridor.
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


def stop(n, msg=None):
    """Zero the setpoint and keep publishing it -- one Twist can be missed."""
    for _ in range(5):
        n.drive(0.0)
        n.spin(0.12)
    if msg:
        print(msg)


def fail(n, msg, code=2):
    stop(n)
    print(f'⛔ {msg}')
    rclpy.shutdown()
    sys.exit(code)


def fit_ramp(samples, key):
    """Least-squares slope+intercept of `key` against time, over MOVING samples.

    The discriminator: H1 predicts a positive slope of roughly
    RO_SPEED_I * error, H2 predicts a slope of zero and an offset instead. Only
    samples where the wheels are actually turning are used -- throttle before
    the rover breaks static friction is not the controller tracking anything.
    """
    pts = [(s['t'], s[key]) for s in samples
           if s['rpm'] > 0 and s[key] is not None]
    if len(pts) < 8:
        return None
    n = len(pts)
    tm = sum(p[0] for p in pts) / n
    ym = sum(p[1] for p in pts) / n
    den = sum((p[0] - tm) ** 2 for p in pts)
    if den <= 0:
        return None
    slope = sum((p[0] - tm) * (p[1] - ym) for p in pts) / den
    return dict(slope=slope, intercept=ym - slope * tm, n=n,
                first=pts[0][1], last=pts[-1][1],
                span_s=pts[-1][0] - pts[0][0])


def bumper_speed(moving, window=0.5):
    """Speed from the closing bumper range: a ruler INDEPENDENT of wheel odometry.

    /odom is derived from ERPM and is exactly what is under suspicion, so every
    conclusion that rests on it alone is circular. /scan reads 1.55% short
    (autonav_reference §5), corrected here. Returns per-window m/s.
    """
    out, i = [], 0
    while i < len(moving):
        t0 = moving[i]['t']
        w = [x for x in moving[i:] if x['t'] < t0 + window]
        if len(w) < 5 or (w[-1]['t'] - w[0]['t']) < 0.3 * window:
            break
        if w[0].get('bumper') is None or w[-1].get('bumper') is None:
            return []
        dt = w[-1]['t'] - w[0]['t']
        out.append((w[0]['bumper'] - w[-1]['bumper']) / dt * 1.0155)
        i += len(w)
    return out


def _bumper_lurching(moving):
    """True only if the INDEPENDENT ruler shows real lurching. See the caller."""
    bs = [b for b in bumper_speed(moving) if b > 0]
    if len(bs) < 4:
        return False, 'no usable bumper trace - stick-slip NOT judged'
    mb = sum(bs) / len(bs)
    dips = sum(1 for b in bs if b < 0.5 * mb)
    if dips > 0.15 * len(bs):
        return True, f'bumper shows {100*dips/len(bs):.0f}% of windows below half the mean'
    return False, f'bumper is steady ({100*dips/len(bs):.0f}% of windows below half the mean)'


def analyse(path):
    d = json.load(open(path))
    cmd = d['commanded']
    ff = cmd / RO_MAX_THR_SPEED
    s = d['samples']
    moving = [x for x in s if x['rpm'] > 0]

    print(f"\n--- {path} ---")
    print(f"  commanded            {cmd:.3f} m/s")
    print(f"  feedforward throttle {ff:.3f}   (= {cmd:.3f} / RO_MAX_THR_SPEED {RO_MAX_THR_SPEED})")
    if not moving:
        print('  WHEELS NEVER TURNED - no result.')
        return
    thr = [x['thr'] for x in moving if x['thr'] is not None]
    odo = [abs(x['odom_v']) for x in moving]
    ekf = [x['ekf_v'] for x in moving if x['ekf_v'] is not None]
    if thr:
        print(f"  throttle observed    {min(thr):.3f} -> {max(thr):.3f}  (mean {sum(thr)/len(thr):.3f})")
        print(f"    saturated (>=0.99) on {sum(1 for t in thr if t >= 0.99)}/{len(thr)} samples")
    if odo:
        print(f"  /odom speed          {min(odo):.3f} -> {max(odo):.3f}  (mean {sum(odo)/len(odo):.3f})")
        print(f"    overspeed ratio    {(sum(odo)/len(odo))/cmd:.2f}x the command")
    if ekf:
        below = sum(1 for v in ekf if v < RO_SPEED_TH)
        print(f"  EKF |v| (feedback)   {min(ekf):.3f} -> {max(ekf):.3f}  (mean {sum(ekf)/len(ekf):.3f})")
        print(f"    below RO_SPEED_TH ({RO_SPEED_TH}) on {below}/{len(ekf)} samples"
              f"  -> controller saw EXACTLY ZERO on those")
    print(f"  v_xy_valid ever true: {d['ekf_valid_ever']}   dead_reckoning always: {d['ekf_dr_always']}")

    # The independent ruler. Reported BEFORE the verdict because the overspeed
    # ratio computed off /odom understates the real one by the odom under-read.
    bs = [b for b in bumper_speed(moving) if b > 0]
    if len(bs) >= 4:
        mb = sum(bs) / len(bs)
        print(f"  bumper speed (INDEPENDENT of /odom)  mean {mb:.3f} m/s"
              f"   -> TRUE overspeed {mb/cmd:.2f}x the command")
        if odo:
            mo_ = sum(odo) / len(odo)
            print(f"    /odom under-reads by {100*(1-mo_/mb):.1f}% against it"
                  f"  (mean {mo_:.3f} vs {mb:.3f})")

    r = fit_ramp(s, 'thr')
    print('\n  VERDICT')
    if r is None:
        print('    too few moving samples to fit a ramp - run longer.')
        return

    # H1 is a RAMP in throttle over time: the integral climbing because the
    # feedback reads zero. Rate is RO_SPEED_I * error, and with the feedback
    # deadbanded to zero the error is the whole setpoint.
    expected_i = RO_SPEED_I * cmd
    ramping = expected_i > 0 and r['slope'] > 0.3 * expected_i
    print(f"    throttle slope   {r['slope']:+.4f} /s over {r['span_s']:.1f} s"
          f"   (H1 windup predicts ~{expected_i:+.4f}/s)")

    # H2 canNOT be read off the throttle number. The feedforward is computed
    # from RO_MAX_THR_SPEED, so a wrong RO_MAX_THR_SPEED moves the throttle and
    # the predicted feedforward by exactly the same amount and cancels. It shows
    # up only in what the throttle BUYS: speed achieved per unit throttle is the
    # true full-throttle speed, which is what RO_MAX_THR_SPEED claims to be.
    #
    # ...but ONLY where speed is roughly proportional to throttle. Near the
    # bottom of the range it is not, and the ratio explodes. The 2026-08-12 run
    # produced "implied full-throttle speed >= 4.32 m/s" on a rover that reaches
    # about 0.6, and printed it as a recommendation to raise RO_MAX_THR_SPEED --
    # which would have gutted the feedforward. The estimate is now REFUSED
    # unless the throttle is high enough for the linear model to mean anything.
    mt = sum(thr) / len(thr) if thr else 0.0
    mo = sum(odo) / len(odo) if odo else 0.0

    # Stick-slip: does the rover LURCH rather than creep? Tempting to read that
    # off /odom -- and wrong. On the 2026-08-12 run /odom swung with CoV 0.30
    # and dipped to 0.070 while the bumper, an INDEPENDENT ruler, showed a
    # steady 0.13-0.15 (CoV 0.124). The rover was not lurching at all: ERPM is
    # coarsely quantised at low rpm and /odom inherits every step of it.
    # Diagnosing a mechanical fault off that would have been a fabricated story.
    # Stick-slip is therefore judged ONLY against the bumper, and where there is
    # no usable bumper trace it is NOT judged at all.
    stick_slip, slip_why = _bumper_lurching(moving)

    implied = None
    if mt >= THROTTLE_FLOOR_FOR_IMPLIED and not stick_slip:
        implied = mo / mt
        print(f"    speed per throttle {implied:.3f} m/s at full throttle (implied)"
              f"   vs RO_MAX_THR_SPEED {RO_MAX_THR_SPEED}")
        print(f"      ^ from /odom, which UNDER-reads ~22% at crawl -> this is a FLOOR")
    else:
        why = (f'mean throttle {mt:.3f} is below the {THROTTLE_FLOOR_FOR_IMPLIED:.2f} floor'
               if mt < THROTTLE_FLOOR_FOR_IMPLIED else slip_why)
        print(f"    implied full-throttle speed: REFUSED ({why}).")
        print( "      Speed is not proportional to throttle down here, so the ratio is")
        print( "      meaningless - it reads absurdly high. Do NOT raise RO_MAX_THR_SPEED on it.")

    # 1.25x leaves room for the odom under-read to work against detection rather
    # than for it: the bias makes `implied` too SMALL, so exceeding the threshold
    # anyway is the conservative direction.
    ff_wrong = implied is not None and implied > 1.25 * RO_MAX_THR_SPEED
    # The controller cut throttle well below feedforward => it SAW the overspeed
    # and fought it. That exonerates the loop and points at the drivetrain.
    fighting = mt < 0.6 * ff

    if ramping:
        print('    => H1 INTEGRAL WINDUP. The feedback is dead; the loop is running open.')
        print('       Fix: RO_SPEED_I = 0 (and likely RO_SPEED_P = 0) until the EKF')
        print('       velocity is real. That makes speed open-loop but PREDICTABLE.')
        if ff_wrong:
            print('       ALSO: RO_MAX_THR_SPEED understates the real full-throttle speed.')
    elif stick_slip and fighting:
        print('    => H3 MECHANICAL FLOOR. The loop is NOT at fault: it cut throttle to')
        print(f'       {mt:.3f}, well under the {ff:.3f} feedforward, so it saw the overspeed')
        print('       and fought it. The rover lurches because it is below the slowest')
        print('       speed the drivetrain can hold. This command is not achievable -')
        print('       raise the command, do not retune the controller.')
    elif ff_wrong:
        print('    => H2 FEEDFORWARD. Throttle is flat but RO_MAX_THR_SPEED understates')
        print(f'       the true full-throttle speed (>= {implied:.2f} m/s). Correct it.')
    elif implied is None:
        print('    => INCONCLUSIVE at this speed. Throttle is not ramping, so the loop is')
        print('       not winding up, but the run is too far down the drivetrain range to')
        print('       say anything about the feedforward. Re-run at a higher command.')
    else:
        print('    => NEITHER. Throttle is flat AND buys about the speed'
              ' RO_MAX_THR_SPEED claims,')
        print('       so the overspeed is DOWNSTREAM of the speed controller')
        print('       (allocation / ESC / erpm scaling). Do NOT guess - look there next.')

    # The feedback PX4 closes on comes from rover_ekf_bridge, which is fed from
    # /odom -- so "EKF agrees with /odom" is circular and proves nothing about
    # truth. Both inherit the same scale error, and the controller regulates to
    # a speed that reads low by that much.
    if ekf and odo:
        print(f"\n  NOTE: EKF feedback {sum(ekf)/len(ekf):.3f} vs /odom {mo:.3f} - these are NOT")
        print( "  independent (the bridge feeds the EKF from /odom). The controller inherits")
        print( "  /odom's under-read, so it holds the rover FASTER than it believes.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.05,
                    help='commanded speed [m/s]. 0.05 is the known-safe crawl.')
    ap.add_argument('--duration', type=float, default=20.0,
                    help='how long to hold the command [s] - the integral needs TIME to show')
    ap.add_argument('--max-travel', type=float, default=1.20,
                    help='/odom travel limit [m] (under-reads ~22%% at crawl - not the primary gate)')
    ap.add_argument('--max-measured-speed', type=float, default=0.20,
                    help='ABORT if /odom shows the rover exceeding this [m/s]')
    ap.add_argument('--min-bumper', type=float, default=0.40,
                    help='ABORT if clearance falls below this [m] - the odom-independent backstop')
    ap.add_argument('--min-corridor', type=float, default=2.0,
                    help='refuse to start unless the corridor is at least this clear [m]')
    ap.add_argument('--spinup', type=float, default=5.0,
                    help='abort if the wheels have not turned by this point [s]')
    ap.add_argument('--out', default=None, help='JSON output path')
    ap.add_argument('--analyse', help='score a JSON from a previous run and exit')
    a = ap.parse_args()

    if a.analyse:
        analyse(a.analyse)
        return 0

    # The EKF velocity IS the thing under test. Without the bridge it is dead for
    # a reason that has nothing to do with the speed controller, and the run
    # would confirm H1 for the wrong reason.
    br = subprocess.run(['systemctl', 'is-active', 'rover-ekf-bridge'],
                        capture_output=True, text=True).stdout.strip()
    if br != 'active':
        print(f'⛔ rover-ekf-bridge is {br}, not active.')
        print('   Without it PX4 has no velocity estimate at all and this run proves nothing.')
        print('   Start it (FLOOR ONLY - wheels-up is a limit-cycle hazard):')
        print('     printf \'1987\\n\' | sudo -S systemctl start rover-ekf-bridge')
        return 2

    rclpy.init()
    n = SpeedProbe()
    n.spin(6.0)

    if n.arming is None:
        fail(n, 'no vehicle_status - is the DDS agent up?')
    if n.scan_t is None:
        fail(n, 'no /scan - the clearance backstop would not exist')
    if n.odom_x is None:
        fail(n, 'no /odom - ground truth missing, which is the point of this test')
    if n.thr_n == 0:
        fail(n, 'no /fmu/out/rover_throttle_setpoint - the controller output is THE measurement. '
                'Check dds_topics.yaml:111 is in the flashed build.')
    if n.arming != ARM_ARMED:
        fail(n, 'NOT ARMED. Arm in Manual via RC first; this script never arms.', 1)
    if n.max_rpm() != 0:
        fail(n, f'WHEELS ALREADY TURNING (rpm {n.max_rpm()}). Aborting.', 1)
    if n.valid is not None and n.valid < 0.35:
        fail(n, f'scan validity {100*n.valid:.1f}% is below the mode gate (35%) - it would block blind.')

    b0 = n.bumper
    print(f'state: arming={n.arming} nav={n.nav} rpm=0 validity={100*n.valid:.1f}%')
    print(f'EKF: v_xy_valid={n.ekf_valid} dead_reckoning={n.ekf_dr} |v|={n.ekf_v:.3f} m/s')
    print(f'corridor clearance {b0:.2f} m at the bumper')

    # Opposite of the standoff test: that one REQUIRES a wall, this one requires
    # the absence of one. Nothing here stops the rover but the gates.
    if math.isfinite(b0) and b0 < a.min_corridor:
        fail(n, f'corridor only {b0:.2f} m clear, need >= {a.min_corridor:.2f} m. '
                f'This test has no wall to stop it - it needs room.')

    print('\nswitching to AutoNav ...')
    for _ in range(5):
        n.set_mode(MAIN_CUSTOM, SUB_AUTONAV)
        n.spin(0.4)
    n.spin(2.0)
    if n.nav != NAV_AUTONAV:
        fail(n, f'AutoNav did not hold (nav={n.nav}). No motion. Aborting.', 1)
    print(f'AutoNav holding (nav={n.nav}).')

    print(f'\n>>> HOLDING {a.speed} m/s for {a.duration:.0f} s <<<')
    print(f'    hand on ch8. gates: {a.max_measured_speed:.2f} m/s measured, '
          f'{a.min_bumper:.2f} m clearance, {a.max_travel:.2f} m travel\n')
    print(f"{'t(s)':>6} {'thr':>6} {'ekf|v|':>7} {'odom v':>7} {'travel':>7} "
          f"{'setpt':>6} {'rpm':>5} {'clear':>6}")
    print('-' * 60)

    t0 = time.time()
    nxt = 0.0
    reason = 'duration'
    ekf_valid_ever = False
    ekf_dr_always = True

    while True:
        n.drive(a.speed)
        rclpy.spin_once(n, timeout_sec=0.02)
        el = time.time() - t0

        if n.ekf_valid:
            ekf_valid_ever = True
        if n.ekf_dr is False:
            ekf_dr_always = False

        if n.arming != ARM_ARMED:
            reason = 'DISARMED mid-run'
            break
        if n.nav != NAV_AUTONAV:
            reason = f'AutoNav dropped (nav={n.nav})'
            break

        n.samples.append(dict(t=el, thr=n.thr, ekf_v=n.ekf_v, odom_v=n.odom_vx,
                              travel=n.travel(), sp=n.sp, rpm=n.max_rpm(),
                              bumper=(n.bumper if math.isfinite(n.bumper) else None)))

        if el >= nxt:
            nxt += 1.0
            cl = f'{n.bumper:6.2f}' if math.isfinite(n.bumper) else '   inf'
            thr = f'{n.thr:6.3f}' if n.thr is not None else '     -'
            print(f'{el:6.1f} {thr} {n.ekf_v:7.3f} {n.odom_vx:7.3f} '
                  f'{n.travel():7.3f} {(n.sp if n.sp is not None else float("nan")):6.3f} '
                  f'{n.max_rpm():5d} {cl}')

        # Below the friction deadband the rover never moves and the log is all
        # pre-motion throttle, which measures nothing. Say so rather than
        # producing a confident-looking non-result.
        if el >= a.spinup and n.max_rpm() == 0:
            stop(n)
            print(f'\nWHEELS NEVER TURNED in {a.spinup:.1f}s at {a.speed:.3f} m/s.')
            print('  Below the friction deadband. INCONCLUSIVE - raise --speed.')
            rclpy.shutdown()
            sys.exit(3)

        # GATE ON MEASURED SPEED, NEVER ON THE COMMAND.
        # 2026-08-12: commanded 0.25 produced ~0.9 m/s and the rover hit a wall.
        # The command is not a speed - that is the whole subject of this test, so
        # it is the one number that must never be trusted to bound the run.
        if abs(n.odom_vx) > a.max_measured_speed:
            stop(n)
            print(f'\n⛔ ABORT: measured {abs(n.odom_vx):.2f} m/s exceeds the '
                  f'{a.max_measured_speed:.2f} m/s limit (commanded {a.speed:.2f}).')
            reason = 'MEASURED SPEED LIMIT'
            break
        # Clearance does not depend on odom scale, so this is the guard that
        # still works when the odometry is lying - which at crawl it is, by ~22%.
        if math.isfinite(n.bumper) and n.bumper < a.min_bumper:
            stop(n)
            print(f'\n⛔ ABORT: clearance {n.bumper:.3f} m fell below the '
                  f'{a.min_bumper:.2f} m backstop.')
            reason = 'MIN BUMPER BACKSTOP'
            break
        if n.travel() > a.max_travel:
            stop(n)
            reason = 'max travel'
            break
        if el > a.duration:
            stop(n)
            break

    stop(n)
    n.spin(1.5)

    out = a.out or f'/home/roz/speed_cmd_{time.strftime("%Y%m%d_%H%M%S")}.json'
    rec = dict(commanded=a.speed, reason=reason, samples=n.samples,
               ro_max_thr_speed=RO_MAX_THR_SPEED, ro_speed_p=RO_SPEED_P,
               ro_speed_i=RO_SPEED_I, ro_speed_th=RO_SPEED_TH,
               ekf_valid_ever=ekf_valid_ever, ekf_dr_always=ekf_dr_always,
               final_travel=n.travel())
    with open(out, 'w') as f:
        json.dump(rec, f)
    print(f'\nended: {reason}   travel {n.travel():.3f} m   log -> {out}')
    print('⚠ DISARM before touching the rover. This script never disarms.')

    analyse(out)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
