#!/usr/bin/env python3
"""Field measurement harness — measures, computes, SUGGESTS. Never commands, never writes.

FOR USE WITHOUT ME. Every routine states what it needs, refuses clearly when
something is missing, tells you exactly what to do with the sticks, and ends with
a suggested value and the evidence behind it.

⛔⛔ THIS TOOL NEVER MOVES THE ROVER AND NEVER WRITES A PARAMETER.
  You drive; it watches. That is deliberate: commanding 0.0 m/s does NOT stop
  this rover (measured twice, 100+ rpm after the command), it has overshot a
  distance command by ~5.5x, and S1 is still inconclusive. A tool that drove
  itself would be the dangerous version of this idea. The worst this one can do
  is print a wrong number.

⚠️ ALWAYS: floor, hand on the KILL SWITCH — that is **ch12**, not ch8 — and
  Manual mode. Manual is open loop (the firmware copies the stick straight to
  throttle), so the whole PX4 speed loop is bypassed and none of the RO_* values
  affect what you measure.

ROUTINES
  preflight   is the chain actually alive? Run this first, always.
  scale       validate erpm_to_ms against /scan — needs NO tape measure
  deadband    the stick fraction below which nothing moves
  dropout     the ERPM below which an ESC reports a false zero
  coast       deceleration with the throttle released (no braking)
  brake       deceleration with the RC brake held (ch3)
  ladder      throttle -> speed, loaded. THE G2 measurement.

  python3 tools/field_measure.py preflight
  python3 tools/field_measure.py ladder --duration 300
  python3 tools/field_measure.py scale --analyse run.json

🔴 WHAT THE LADDER CANNOT TELL YOU ON STANDS. The ESCs take throttle as a
  CURRENT (torque) command, so with the wheels in the air anything above
  friction runs to the same no-load ceiling and speed reads FLAT across the
  whole stick. Measured 2026-09-12: ~1505 ERPM from 0.148 stick to full. The
  ladder must be run LOADED, on the floor, or it measures nothing.
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from px4_msgs.msg import (EscStatus, ManualControlSetpoint, RoverThrottleSetpoint,
                          VehicleStatus)
from sensor_msgs.msg import LaserScan

import rover_diag as rd

SCAN_SECTOR_DEG = 20.0


# --------------------------------------------------------------------------
# Recording
# --------------------------------------------------------------------------

class FieldRecorder(Node):
    """Read-only: subscribes, never publishes. One sample per esc_status."""

    def __init__(self):
        super().__init__('field_measure')
        self.samples = []
        self.throttle = self.stick = None
        self.nav = self.arm = None
        self.online = 0
        self.odom_vx = None
        self.scan_min = None
        self.create_subscription(EscStatus, '/fmu/out/esc_status',
                                 self._esc, rd.PX4_QOS)
        self.create_subscription(RoverThrottleSetpoint,
                                 '/fmu/out/rover_throttle_setpoint',
                                 self._thr, rd.PX4_QOS)
        self.create_subscription(ManualControlSetpoint,
                                 '/fmu/out/manual_control_setpoint',
                                 self._man, rd.PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self._status, rd.PX4_QOS)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, rd.SENSOR_QOS)

    def _thr(self, m):
        self.throttle = float(m.throttle_body_x)

    def _man(self, m):
        # In Manual this IS throttle_body_x, and unlike rover_throttle_setpoint
        # it publishes while disarmed, so a run can be rehearsed before arming.
        self.stick = float(m.throttle) if m.valid else None

    def _status(self, m):
        self.nav, self.arm = int(m.nav_state), int(m.arming_state)

    def _odom(self, m):
        self.odom_vx = float(m.twist.twist.linear.x)

    def _scan(self, m):
        half = int(len(m.ranges) * (SCAN_SECTOR_DEG / 2.0)
                   / max(1e-6, (m.angle_max - m.angle_min) * 57.2958))
        mid = len(m.ranges) // 2
        sector = [r for r in m.ranges[max(0, mid - half):mid + half]
                  if r > 0.05 and r < 30.0]
        self.scan_min = min(sector) if sector else None

    def _esc(self, m):
        self.online = int(m.esc_online_flags)
        by_addr = rd.esc_by_address(m)
        if not by_addr:
            return
        self.samples.append({
            't': time.time(),
            'throttle': self.throttle if self.throttle is not None else self.stick,
            'throttle_src': 'fc' if self.throttle is not None else 'stick',
            'rpm': {a: v[0] for a, v in by_addr.items()},      # RAW
            'current': {a: v[1] for a, v in by_addr.items()},
            'online_flags': self.online,
            'nav': self.nav, 'arm': self.arm,
            'odom_vx': self.odom_vx, 'scan_min': self.scan_min,
        })


def record(duration, out_path, banner):
    rclpy.init()
    node = FieldRecorder()
    print(banner)
    print('\n⛔ hand on the KILL SWITCH — ch12.  Ctrl-C stops early.\n')
    print(f'{"t":>7} {"arm":6} {"thr":>6} {"m/s":>7} {"ERPM":>6} '
          f'{"A":>6} {"scan":>6}')
    t0, last = time.time(), 0.0
    try:
        while time.time() - t0 < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.samples or time.time() - last < 0.5:
                continue
            last = time.time()
            s = node.samples[-1]
            v = rd.linear_speed(s['rpm'])
            mag = sum(abs(r) for r in s['rpm'].values()) / len(s['rpm'])
            amp = sum(s['current'].values()) / len(s['current'])
            thr = '  none' if s['throttle'] is None else f"{s['throttle']:+6.3f}"
            speed = f'{v:7.3f}' if v is not None else '    ---'
            scan = f"{s['scan_min']:6.2f}" if s['scan_min'] is not None else '   ---'
            armed = 'ARMED ' if s['arm'] == 2 else 'disarm'
            print(f"{s['t'] - t0:7.1f} {armed:6} {thr} {speed} "
                  f"{mag:6.0f} {amp:6.2f} {scan}", flush=True)
    except KeyboardInterrupt:
        pass

    payload = {'erpm_to_ms': rd.ERPM_TO_MS, 'rpm_is_raw': True,
               'wheel_signs': rd.WHEEL_SIGNS, 'samples': node.samples}
    with open(out_path, 'w') as f:
        json.dump(payload, f)
    print(f'\n{len(node.samples)} samples -> {out_path}')
    node.destroy_node()
    rclpy.shutdown()
    return payload


def load(path):
    """Load a run, tolerating files written by older tools.

    ⚠️ A file without `rpm_is_raw` was recorded while the stale address-10
    inversion was still being applied, so it is un-applied here to recover raw
    ERPM. Without this an old log silently scores at half speed.
    """
    d = json.load(open(path))
    raw = bool(d.get('rpm_is_raw'))
    for s in d['samples']:
        s['rpm'] = {int(a): v for a, v in s['rpm'].items()}
        if not raw:
            s['rpm'] = {a: v / (-1.0 if a == 10 else 1.0) for a, v in s['rpm'].items()}
        s['current'] = {int(a): v for a, v in s['current'].items()}
        s.setdefault('scan_min', None)
        s.setdefault('odom_vx', None)
        s.setdefault('arm', None)
        s.setdefault('throttle', None)
    if not raw:
        print('⚠️ pre-2026-09-12 file: the old addr-10 inversion was un-applied '
              'to recover raw ERPM\n')
    return d


# --------------------------------------------------------------------------
# Requirements
# --------------------------------------------------------------------------

R_ESC = rd.Requirement('/fmu/out/esc_status', EscStatus, 10.0,
                       why='wheel ERPM and current — the whole measurement')
R_ODOM = rd.Requirement('/odom', Odometry, 20.0, why='cross-check against ERPM')
R_SCAN = rd.Requirement('/scan', LaserScan, 10.0,
                        why='the independent distance ruler, and the runway check')
R_STATUS = rd.Requirement('/fmu/out/vehicle_status_v1', VehicleStatus, 0.5,
                          why='arming state, so a run cannot be logged as valid '
                              'while disarmed')

NEEDS = {
    'preflight': [R_ESC, R_ODOM, R_SCAN, R_STATUS],
    'scale': [R_ESC, R_SCAN, R_STATUS, R_ODOM],
    'deadband': [R_ESC, R_STATUS],
    'dropout': [R_ESC, R_STATUS],
    'coast': [R_ESC, R_SCAN, R_STATUS],
    'brake': [R_ESC, R_SCAN, R_STATUS],
    'ladder': [R_ESC, R_SCAN, R_STATUS, R_ODOM],
    'range': [R_ESC, R_STATUS],
    'wheelmap': [R_ESC],          # hand-spun: no arming, no /scan, no runway
    'dimensions': [],             # reads declared constants only
}


def gate(routine, seconds=5.0):
    print(f'\nPREFLIGHT for `{routine}` — measuring for {seconds:.0f}s '
          f'(rates, not `is-active`)')
    ok, lines = rd.check_requirements(NEEDS[routine], duration=seconds)
    for line in lines:
        print(line)
    if not ok:
        print('\n🔴 REFUSING TO RUN. Fix the FAIL lines above first.')
    else:
        print('\n✅ chain is live.')
    return ok


# --------------------------------------------------------------------------
# Routines
# --------------------------------------------------------------------------

def an_scale(d):
    """erpm_to_ms against /scan. No tape measure needed.

    /scan is tape-verified to 4.6 mm over 1.46 m (2026-09-11), so the change in
    range while driving at a flat wall is an independent ruler for the distance
    the wheels claim to have covered.
    """
    seg = [s for s in d['samples']
           if s['scan_min'] is not None and s['arm'] == 2]
    if len(seg) < 20:
        print('not enough armed samples with /scan — drive AT a flat wall')
        return
    # ⛔ SCORE ONE BURST, NOT THE WHOLE FILE. A session normally contains a run
    # out and a drive back to the start; first-to-last then cancels the /scan
    # travel to ~0 while the ERPM integral keeps adding, which on 2026-09-12
    # produced "scan -0.006 m, erpm 15.281 m" from a run that was actually clean.
    # Segments are split on pauses and the LONGEST single approach is scored.
    bursts, cur = [], []
    for s in seg:
        moving = sum(abs(r) for r in s['rpm'].values()) / len(s['rpm']) > 30
        if moving:
            cur.append(s)
        else:
            if len(cur) >= 5:
                bursts.append(cur)
            cur = []
    if len(cur) >= 5:
        bursts.append(cur)
    if not bursts:
        print('the wheels never turned — nothing to scale')
        return

    def travel(b):
        return (b[0]['scan_min'] - b[-1]['scan_min']) * rd.SCAN_SCALE

    # The best burst is the longest APPROACH: /scan must shrink, so the camera
    # is closing on the surface it is ranging.
    approaches = [b for b in bursts if travel(b) > 0.05]
    if not approaches:
        print(f'\n  {len(bursts)} burst(s), none of them an approach.')
        print('  /scan has to SHRINK for this to work — drive TOWARDS the wall, '
              'not away from it.')
        return
    moving = max(approaches, key=travel)
    if len(bursts) > 1:
        print(f'\n  {len(bursts)} bursts in this file; scoring the longest '
              f'approach only ({travel(moving):.2f} m).')

    # 🔴 THE RULER HAS TO BE TRACKING BEFORE IT CAN RULE. On 2026-09-12 the first
    # 1.9 s of a clean burst showed /scan FLAT at ~3.8 m (drifting UP, even) while
    # the wheels integrated 1.7 m: at that range the 0.275 m corridor subtends
    # only ±4 deg, and the far wall was at the edge of the 5 m limit, so the
    # sector minimum was a marginal return that did not follow the rover. Scoring
    # that window in charged the wheels for distance the ruler never saw and gave
    # a 36% error where the tracking window gives 8%.
    # Walk BACK from the end while /scan keeps closing. The moment it stops
    # falling for a sustained stretch, the ruler was not tracking before that,
    # and everything earlier is discarded.
    STALL = 0.30            # [s] of no net closing = the ruler was not tracking
    start = 0
    for i in range(len(moving) - 1, 0, -1):
        j = i
        while j > 0 and moving[i]['t'] - moving[j]['t'] < STALL:
            j -= 1
        if moving[j]['scan_min'] - moving[i]['scan_min'] <= 0.01:
            start = i
            break
    dropped = start
    moving = moving[start:]
    if len(moving) < 5:
        print('\n  the tracking window is too short to score.')
        return
    if dropped:
        print(f'\n  ⚠️ dropped the first {dropped} samples: /scan was NOT '
              f'tracking yet (flat near {moving[0]["scan_min"]:.2f} m — at that '
              'range the')
        print('     0.275 m corridor subtends only a few degrees and the wall '
              'sits at the edge of')
        print('     the 5 m limit, so the sector minimum does not follow the '
              'rover). Scoring only')
        print('     the window where the ruler was actually closing.')

    first, last = moving[0], moving[-1]
    scan_travel = travel(moving)
    odo = 0.0
    for a, b in zip(moving, moving[1:]):
        v = rd.linear_speed(a['rpm'])
        if v is not None:
            odo += v * (b['t'] - a['t'])
    if abs(odo) < 0.05 or abs(scan_travel) < 0.05:
        print(f'travel too small to judge (scan {scan_travel:.3f} m, '
              f'erpm {odo:.3f} m) — drive at least 0.5 m')
        return

    ratio = scan_travel / odo
    print(f'\n  /scan travel      : {scan_travel:+.3f} m   '
          f'({first["scan_min"]:.3f} -> {last["scan_min"]:.3f} raw)')
    print(f'  ERPM-integrated   : {odo:+.3f} m')
    print(f'  ratio scan/erpm   : {ratio:.4f}   '
          f'({"/odom over-reads" if ratio < 1 else "/odom under-reads"} '
          f'by {abs(1 - ratio) * 100:.1f}%)')
    rd.suggest('erpm_to_ms (ROS param, not a PX4 one)',
               f'{rd.ERPM_TO_MS * ratio:.6f}',
               f'{scan_travel:.3f} m by /scan vs {odo:.3f} m integrated from ERPM',
               ['/scan is the ruler here; it is verified to ~5 mm at 1.5 m.',
                'erpm_to_ms is SPEED-DEPENDENT — quote the speed this was run at.',
                'This lives in wheel_odometry_node.py, NOT on the FC.',
                'Since 2026-09-12 /odom reads ~2x what it used to (sign fix) — '
                'a ratio near 1.0 CONFIRMS that fix.'],
               px4_param=False)


def an_deadband(d):
    segs = rd.steady_segments(d['samples'], 'throttle', 0.02, 0.8)
    moved, still = [], []
    for seg in segs:
        thr = abs(sum(s['throttle'] for s in seg) / len(seg))
        mags = [sum(abs(r) for r in s['rpm'].values()) / len(s['rpm']) for s in seg]
        amps = [c for s in seg for c in s['current'].values()]
        mag = sum(mags) / len(mags)
        # ⛔ A wheel still spinning after the stick was centred is a COAST, not
        # evidence that this stick level moves the rover. is_settled() rejects
        # those (negative current = regenerating). Without this the routine
        # reported a deadband of 0.000 off a spin-down.
        if not rd.is_settled(mags, amps):
            continue
        if mag > 30 and thr > 0.01:
            moved.append((thr, mag))
        elif mag <= 30:
            still.append((thr, mag))
    if not moved:
        print('\n  no stick level produced sustained DRIVEN motion — push further,')
        print('  and hold each level still for at least a second.')
        return
    lowest_moving = min(m[0] for m in moved)
    highest_still = max((s[0] for s in still), default=0.0)
    print(f'\n  highest stick with NO motion : {highest_still:.3f}')
    print(f'  lowest stick WITH motion     : {lowest_moving:.3f}')

    if highest_still >= lowest_moving:
        # ⛔ The bracket is inverted: some higher stick produced no motion while
        # a lower one did. Averaging two numbers that contradict each other
        # would launder a data problem into a confident-looking answer.
        print('\n🔴 THE BRACKET IS INCONSISTENT — a HIGHER stick produced no motion')
        print('   than one that did. That is not a deadband, it is a bad run:')
        print('   usually a level caught mid-transition, or a segment recorded '
              'while disarmed.')
        print('   ⛔ No value suggested. Re-run, holding each level still for '
              '2 s, armed throughout.')
        return
    rd.suggest('friction deadband (stick fraction)',
               f'{(highest_still + lowest_moving) / 2:.3f}',
               f'bracketed between {highest_still:.3f} and {lowest_moving:.3f}',
               ['Below this the rover sits and grinds — never command into it.',
                'On stands this reads LOWER than on the floor (no load).'],
               px4_param=False)


def an_dropout(d):
    hits = [s for s in d['samples']
            if any(abs(r) < 1e-6 for r in s['rpm'].values())
            and any(abs(r) > 50 for r in s['rpm'].values())]
    if not hits:
        print('\n  no zero-dropout seen in this run.')
        return
    floor = min(max(abs(r) for r in s['rpm'].values()) for s in hits)
    print(f'\n  {len(hits)} samples where one wheel read EXACTLY 0 '
          f'while another turned')
    print(f'  lowest co-occurring ERPM: {floor:.0f} '
          f'({floor * rd.ERPM_TO_MS:.3f} m/s)')
    print('  🔑 This is G2 item (a). It is why a /odom-based safety backstop is '
          'useless at crawl —')
    print('     gate on /scan clearance instead.')


MIN_DECEL_SPAN = 0.30      # [s] shorter than this and ESC noise dominates


def _decel(d, label):
    """Slope of the longest sustained slow-down, fitted — NOT the steepest pair.

    🔴 The obvious implementation (largest sample-to-sample drop) is wrong and
    was caught by a smoke test: at ~80 Hz a single noisy ERPM sample yields
    87 m/s², which is nine g and would have been printed as a brake figure.
    A deceleration must be fitted over a span, and the span must be long enough
    that noise cannot dominate it.
    """
    # ⚠️ ERPM IS QUANTISED AND NOISY AT ~100 Hz, so a decelerating run contains
    # small upward blips. Requiring every sample to be slower than the last
    # breaks the run on the first blip: a real 1.6 m/s^2 brake was truncated at
    # 0.57 m/s and scored 0.60 on 2026-09-12. Track the minimum reached and
    # allow this much noise above it.
    NOISE = 0.08                          # [m/s]
    runs, cur, floor = [], [], None
    for s in d['samples']:
        v = rd.linear_speed(s['rpm'])
        if v is None:
            continue
        if cur:
            floor = min(floor, v)
        # ⛔ STOP THE RUN AT REST. A stationary tail satisfies "slower than the
        # last sample" forever (0 < 0 + 0.01), so the run used to swallow the
        # zeros and the fit was dragged down by a segment that was mostly flat:
        # a real 1.6 m/s^2 brake scored 0.60 on 2026-09-12. Once it is stopped,
        # the deceleration is over.
        if cur and v <= floor + NOISE and v > 0.05:
            cur.append((s['t'], v))
        else:
            if v <= 0.05 and cur:
                cur.append((s['t'], v))       # keep the one sample AT rest
            if len(cur) > 3 and cur[-1][0] - cur[0][0] >= MIN_DECEL_SPAN:
                runs.append(cur)
            cur = [(s['t'], v)] if v > 0.15 else []
            floor = v if cur else None
    if len(cur) > 3 and cur[-1][0] - cur[0][0] >= MIN_DECEL_SPAN:
        runs.append(cur)

    runs = [r for r in runs if r[0][1] - r[-1][1] > 0.10]
    if not runs:
        print(f'\n  no {label} segment found that lasted >= {MIN_DECEL_SPAN:.2f} s '
              f'with a real speed drop.')
        print('  Spin up further, then release cleanly and let it run all the way '
              'down.')
        return
    run = max(runs, key=lambda r: r[-1][0] - r[0][0])
    t0 = run[0][0]
    k, _ = rd.linfit([(t - t0, v) for t, v in run])
    if k is None or k >= 0:
        print(f'\n  {label}: could not fit a slope.')
        return
    rate, span = -k, run[-1][0] - run[0][0]
    print(f'\n  {label} deceleration: {rate:.2f} m/s²  '
          f'(fitted over {span:.2f} s, {run[0][1]:.2f} -> {run[-1][1]:.2f} m/s, '
          f'{len(run)} samples)')
    print(f'  implied stopping distance from 0.8 m/s: '
          f'{0.8 ** 2 / (2 * rate):.2f} m')
    if run[-1][1] > 0.20:
        print('  ⚠️ THIS RUN DID NOT REACH A STOP — the slope is from the fast part '
              'only and')
        print('     will overstate a full stop. That is exactly why the coast '
              'baseline is still provisional.')
    print('  ⚠️ n=1 from this run. Repeat before quoting a ratio.')


def an_coast(d):
    _decel(d, 'coast')
    print('  ⛔ Zero throttle is a FREE COAST, not braking. This number is why '
          'the standoff is speed-bound.')


def an_brake(d):
    _decel(d, 'brake')
    print('  🔑 Compare against the 0.69 m/s² measured loaded on 2026-09-09.')


def an_ladder(d):
    segs = rd.steady_segments(d['samples'], 'throttle', 0.02, 1.0)
    points = []
    print(f'\n{"thr":>7} {"m/s":>8} {"ERPM":>7} {"A":>7}   ratio')
    for seg in segs:
        thr = sum(s['throttle'] for s in seg) / len(seg)
        if abs(thr) < 0.01:
            continue
        mags = [sum(abs(r) for r in s['rpm'].values()) / len(s['rpm']) for s in seg]
        amps = [c for s in seg for c in s['current'].values()]
        if not rd.is_settled(mags, amps):
            continue
        speeds = [v for v in (rd.linear_speed(s['rpm']) for s in seg) if v is not None]
        if not speeds:
            continue
        v = sum(speeds) / len(speeds)
        print(f'{thr:+7.3f} {v:8.3f} {sum(mags)/len(mags):7.0f} '
              f'{sum(amps)/len(amps):7.2f}   {v/thr:7.3f}')
        points.append((thr, v))

    moving = [p for p in points if abs(p[1]) > 0.05]
    if len(moving) < 2:
        print('\n  not enough held rungs — hold each level 2 s, centre, step up')
        return
    spread = ((max(p[1] for p in moving) - min(p[1] for p in moving))
              / max(p[1] for p in moving))
    if spread < 0.10 and max(p[0] for p in moving) > 2 * min(p[0] for p in moving):
        print('\n🔴 SPEED IS FLAT ACROSS THE THROTTLE RANGE.')
        print('   Throttle is not setting speed here. That is a TORQUE command '
              'with nothing to push')
        print('   against — the signature of a wheels-up run. ⛔ No fit is '
              'meaningful. Go to the floor.')
        return

    k, c = rd.linfit(moving)
    if k is None:
        return
    full = k + c
    print(f'\n  fit: speed = {k:.3f} x throttle {c:+.3f}')
    rd.suggest('RO_MAX_THR_SPEED', f'{full:.2f}',
               f'{len(moving)} loaded rungs, extrapolated to full throttle',
               ['🔴 THE ESCs ARE IN CURRENT (TORQUE) MODE, so this value is '
                'SURFACE-SPECIFIC.',
                'PX4 assumes throttle is proportional to speed, which is a '
                'duty-mode property.',
                'Expect a different answer on carpet or a slope. Record the '
                'surface with the number.',
                'If the ratio column is not flat, something else is in the path '
                'too — do not apply.'])


def an_range(d):
    """Throttle OUTPUT range: where does it start biting, where does it stop growing?

    This is the rover equivalent of the drone's idle/min/max actuator setup,
    which QGC does not offer for rovers. It is about the OUTPUT SCALING
    (UAVCAN_EC_MIN/MAX), not about speed.

    🔑 UAVCAN_EC_MIN1..4 = 110 and MAX = 8082 ARE DELIBERATE. ⛔ Never "tidy"
    them back to 10/8191 — a raw command of 10 is dead full-reverse on these
    ESCs. This routine reports what the configured range actually buys; it does
    not propose changing MIN/MAX.
    """
    segs = rd.steady_segments(d['samples'], 'throttle', 0.02, 0.8)
    rows = []
    for seg in segs:
        thr = sum(s['throttle'] for s in seg) / len(seg)
        mags = [sum(abs(r) for r in s['rpm'].values()) / len(s['rpm']) for s in seg]
        amps = [c for s in seg for c in s['current'].values()]
        if not rd.is_settled(mags, amps):
            continue
        signed = [sum(s['rpm'].values()) / len(s['rpm']) for s in seg]
        rows.append((thr, sum(mags) / len(mags), sum(amps) / len(amps),
                     sum(signed) / len(signed)))
    if not rows:
        print('\n  no held levels — hold each step still for at least a second')
        return

    rows.sort()
    print(f'\n{"stick":>7} {"ERPM":>8} {"A":>7}  note')
    for thr, mag, amp, signed in rows:
        note = ''
        if mag <= 30:
            note = 'no motion (inside the deadband)'
        elif thr > 0 and signed < 0:
            note = '🔴 FORWARD STICK, REVERSE ROTATION'
        elif thr < 0 and signed > 0:
            note = '🔴 REVERSE STICK, FORWARD ROTATION'
        print(f'{thr:+7.3f} {mag:8.0f} {amp:7.2f}  {note}')

    driving = [r for r in rows if r[1] > 30]
    if not driving:
        print('\n  nothing ever drove — push further.')
        return
    idle = min(abs(r[0]) for r in driving)
    top = max(driving, key=lambda r: r[1])
    near_top = [r for r in driving if r[1] > 0.97 * top[1]]
    saturates_at = min(abs(r[0]) for r in near_top)

    print(f'\n  idle / bite point : {idle:.3f} stick — below this, no motion')
    print(f'  peak ERPM         : {top[1]:.0f} at {top[0]:+.3f} stick')
    print(f'  saturates from    : {saturates_at:.3f} stick '
          f'(within 3% of peak from here up)')
    usable = max(0.0, 1.0 - idle)
    print(f'  usable span       : {idle:.3f} .. 1.000  ({usable * 100:.0f}% of travel)')

    if saturates_at < 0.5 * max(abs(r[0]) for r in driving):
        print('\n🔴 IT SATURATES IN THE LOWER HALF OF THE STICK.')
        print('   Wheels-up that is expected and means nothing — a torque command '
              'with no load')
        print('   runs to the same ceiling whatever you ask for. LOADED, on the '
              'floor, it would')
        print('   instead mean the output range is compressed and most of your '
              'stick does nothing.')
    print('\n  ⛔ UAVCAN_EC_MIN1..4=110 / MAX1..4=8082 are DELIBERATE. '
          'Never reset them to 10/8191 —')
    print('     a raw command of 10 is dead full-reverse. Nothing here suggests '
          'changing them.')


def an_wheelmap(d):
    """Which ESC address is which physical corner — verified by HAND, not by command.

    🔑 WHY THIS EXISTS. QGC can spin a drone's motors one at a time to identify
    them; it offers nothing equivalent for a rover. The address-to-corner map
    (10=RF 11=FL 12=RR 13=RL) comes from three paper sources that agree with
    each other, and has NEVER been confirmed against the physical vehicle.

    ⛔ AND IT IS DONE BY HAND ON PURPOSE. PX4 can drive one output at a time via
    actuator test, but that commands a live motor from a laptop. Spinning a
    raised wheel by hand proves the same thing, cannot run away, and needs no
    arming. It also reveals each wheel's SIGN, which is what went stale and
    halved /odom.
    """
    events = []
    for s in d['samples']:
        for addr, rpm in s['rpm'].items():
            if abs(rpm) > 25:
                events.append((s['t'], addr, rpm))
    if not events:
        print('\n  no wheel movement detected at all — spin each wheel firmly.')
        return

    t0 = events[0][0]
    groups, cur = [], [events[0]]
    for e in events[1:]:
        if e[0] - cur[-1][0] > 2.0:          # a gap = the next wheel
            groups.append(cur)
            cur = [e]
        else:
            cur.append(e)
    groups.append(cur)

    print(f'\n  {len(groups)} spin episodes detected\n')
    print(f'{"#":>3} {"start":>7} {"addr":>5} {"peak":>7} {"direction":>10}  share')
    for i, g in enumerate(groups, 1):
        by_addr = {}
        for t, addr, rpm in g:
            by_addr.setdefault(addr, []).append(rpm)
        total = sum(len(v) for v in by_addr.values())
        main = max(by_addr, key=lambda a: len(by_addr[a]))
        peak = max(by_addr[main], key=abs)
        share = len(by_addr[main]) / total
        direction = 'forward' if peak > 0 else 'REVERSE'
        print(f'{i:>3} {g[0][0]-t0:7.1f} {main:5d} {peak:+7.0f} {direction:>10}  '
              f'{share:.0%}')
        if share < 0.8:
            others = ', '.join(str(a) for a in by_addr if a != main)
            print(f'      ⚠️ addresses {others} also moved — belt drag, or two '
                  f'wheels spun at once.')
            print('         Spin ONE wheel at a time, and let the others stop '
                  'fully between.')

    seen = [max(((a, len(v)) for a, v in
                 {addr: [r for _, ad, r in g if ad == addr]
                  for addr in {ad for _, ad, _ in g}}.items()),
                key=lambda x: x[1])[0] for g in groups]
    print(f'\n  order observed: {seen}')
    print('  🔑 Compare against the order you spun them in. The configured map is')
    print('     10=RF  11=FL  12=RR  13=RL  (MOTOR_MAP.md — paper only, never '
          'verified on the vehicle).')
    print('  ⛔ If an address reports REVERSE while you spun it forward, that is a '
          'SIGN problem,')
    print('     not a mapping problem — and it is the failure that halved /odom '
          'until 2026-09-12.')


def an_dimensions(_):
    """Cross-check the declared dimensions against each other. Reads files only.

    It cannot measure the rover. What it can do is catch the constants
    disagreeing, which is exactly how 0.450 and 0.560 both came to be quoted as
    the vehicle's width.
    """
    print('\n  CANONICAL (autonav_reference.md §4)\n')
    for name, (val, why) in rd.DIMENSIONS.items():
        print(f'  {name:18s} {val:7.4f} m   {why}')

    xs = [p[0] for p in rd.NAV2_FOOTPRINT]
    ys = [p[1] for p in rd.NAV2_FOOTPRINT]
    nav_len, nav_wid = max(xs) - min(xs), max(ys) - min(ys)
    print(f'\n  NAV2 FOOTPRINT     {nav_len:.3f} x {nav_wid:.3f} m  '
          f'(front {max(xs):.3f}, rear {min(xs):.3f})')

    ok = True
    for label, nav, key in (('length', nav_len, 'plate_length'),
                            ('width', nav_wid, 'plate_width')):
        ref = rd.DIMENSIONS[key][0]
        if abs(nav - ref) > 0.005:
            ok = False
            print(f'  🔴 {label}: Nav2 {nav:.3f} vs reference {ref:.3f} — DISAGREE')
        else:
            print(f'  ✅ {label}: Nav2 agrees with the reference table')

    front_ref = rd.DIMENSIONS['centre_to_tip'][0]
    if abs(max(xs) - front_ref) > 0.005:
        ok = False
        print(f'  🔴 front extent: Nav2 {max(xs):.3f} vs centre_to_tip '
              f'{front_ref:.3f} — DISAGREE')
    else:
        print('  ✅ front extent matches centre_to_tip')

    lo, hi = rd.DISPUTED_WIDTH
    length = rd.DIMENSIONS['plate_length'][0]
    print(f'\n  ⚠️ WIDTH — TWO FIGURES EXIST: {lo:.3f} vs {hi:.3f} m  (traced 2026-09-12)')
    print(f'     {lo:.3f} — AutoNav Technical Reference §4 (artifact bc09dd55, '
          'rev 08-09), the same')
    print('            table in autonav_reference.md, BOTH Nav2 footprints '
          '(±0.225), the reflex')
    print('            footprint_half_width 0.225, and corridor half-width 0.275 '
          '= 0.225 + 50 mm.')
    print('            That reference states the wheels sit INBOARD of the plate.')
    print(f'     {hi:.3f} — the Solar-Farm Inspection UGV price quotation '
          '(artifact a1f7eb07, Rev 8),')
    print('            "Footprint 730 x 560 mm" — a COMMERCIAL spec, plus the '
          'four docs that')
    print('            copied it: requirements §, MEMORY.md, todos.md, '
          'project_rover_autonav.')
    print(f'     Footprint area: {length * lo:.3f} m² against {length * hi:.3f} m².')
    print('  🔑 The safety code already uses 0.450, so there is NO GAP unless a '
          'tape disagrees.')
    print('  ⏭ Two tape numbers would close it: the plate edge-to-edge, and the '
          'widest point of')
    print('     the whole vehicle. If anything protrudes, THAT is the number Nav2 '
          'and the reflex')
    print('     need. ⛔ Do not resolve it by counting documents — the manual\'s '
          'own rule is')
    print('     "never derive a dimension, look it up", and two bugs have already '
          'shipped here')
    print('     from assumed geometry.')
    print(f'\n  {"✅ no other contradiction found." if ok else "🔴 fix the above."}')


ANALYSERS = {'scale': an_scale, 'deadband': an_deadband, 'dropout': an_dropout,
             'coast': an_coast, 'brake': an_brake, 'ladder': an_ladder,
             'range': an_range, 'wheelmap': an_wheelmap,
             'dimensions': an_dimensions}

BANNERS = {
    'scale': 'Park facing a FLAT WALL with 2 m clear. Drive straight at it for '
             '~1 m at a crawl, then stop well short.',
    'deadband': 'From centre, ease the stick up in small steps, 1 s each, until '
                'the wheels just begin to turn.',
    'dropout': 'Drive as slowly as the rover will go, then slower still. The '
               'point is to catch a wheel reporting a false zero.',
    'coast': 'Spin up to a steady speed, then RELEASE the stick to centre and '
             'let it roll out completely. Touch nothing.',
    'brake': 'Spin up to a steady speed, then apply the RC brake on ch3 and '
             'hold it until fully stopped.',
    'ladder': 'Hold each level 2 s, return to centre, step up: about 0.15, 0.25, '
              '0.40, 0.60, then full if the runway allows.',
    'range': 'Step the stick from centre to full in about 8 even steps, holding '
             'each for 1 s. Then do the same in reverse.',
    'wheelmap': 'WHEELS UP, DISARMED. Spin ONE wheel at a time BY HAND, forwards, '
                'for ~3 s each: front-left, front-right, rear-left, rear-right. '
                'Leave a clear pause between wheels and let each stop fully.',
}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('routine', choices=list(NEEDS))
    ap.add_argument('--duration', type=float, default=180.0)
    ap.add_argument('--out', default=None)
    ap.add_argument('--analyse', help='score a saved run and exit')
    ap.add_argument('--skip-preflight', action='store_true',
                    help='⛔ only when the chain was just verified')
    a = ap.parse_args()

    if a.routine == 'dimensions':
        # Nothing to record: this one reads declared constants, not the vehicle.
        an_dimensions(None)
        return

    if a.analyse:
        ANALYSERS[a.routine](load(a.analyse))
        return

    if not a.skip_preflight and not gate(a.routine):
        raise SystemExit(2)
    if a.routine == 'preflight':
        return

    out = a.out or (f'/home/roz/rover_data/logs/{a.routine}_'
                    f'{time.strftime("%Y%m%d_%H%M%S")}.json')
    d = record(a.duration, out, '\n' + BANNERS[a.routine])
    ANALYSERS[a.routine](d)
    print(f'\nre-score any time: python3 tools/field_measure.py {a.routine} '
          f'--analyse {out}')


if __name__ == '__main__':
    main()
