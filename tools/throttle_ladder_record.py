#!/usr/bin/env python3
"""Open-loop throttle ladder: what speed does FULL THROTTLE actually give?

WHY THIS EXISTS
  PX4 closes the rover speed loop as (RoverControl.cpp:107):

      throttle = setpoint / RO_MAX_THR_SPEED   <- feedforward
               + PID(setpoint - vehicle_speed) <- feedback

  RO_MAX_THR_SPEED is meant to BE the speed at full throttle. It reads 0.60 and
  has never been measured. If it understates the truth, every commanded speed
  comes out scaled up by the same ratio -- which is what the floor has shown:
  two runs on 2026-09-12 commanded 0.10 m/s and measured (by /scan, not /odom)
  2.69 m in 5 s and 1.66 m in 3 s, i.e. ~0.55 m/s both times. 0.10/0.60 = 0.167
  of throttle producing 0.55 m/s implies a true full-throttle speed near 3.3 m/s.
  This tool measures that number instead of inferring it.

  MANUAL MODE IS OPEN LOOP -- verified in the flashed firmware (a52c38b07d):
  DifferentialManualMode::manual() copies the stick straight into
  rover_throttle_setpoint.throttle_body_x. No PID, no RO_MAX_THR_SPEED, and the
  accel slew is off because RO_ACCEL_LIM is -1. So driving the stick in Manual
  and logging ERPM measures the plant with the controller out of the way.

WHEELS-UP VS FLOOR -- THEY MEASURE DIFFERENT THINGS
  On stands there is no load and no slip, so the speed here is WHEEL SURFACE
  SPEED and is an UPPER BOUND on ground speed. It is still worth doing first:
  it costs no floor, it maps the throttle deadband, it shows whether all four
  ESCs wake, and it bounds the answer. ⛔ The number that goes into
  RO_MAX_THR_SPEED must come from a FLOOR run, loaded.
  ⛔ Do NOT start rover-ekf-bridge for a wheels-up run and do NOT engage AutoNav
  on stands -- that pairing is the self-sustaining limit cycle (setup_manual D6).

WHAT IT RECORDS (read-only: never arms, never commands, never disarms)
  /fmu/out/rover_throttle_setpoint  the throttle PX4 actually published
  /fmu/out/esc_status               per-wheel ERPM and current, keyed by ADDRESS
  /fmu/out/input_rc                 stick positions, for context
  /fmu/out/vehicle_status_v1        nav_state + arming, so a run can be audited
  /odom                             the node's own opinion, for cross-check only

TRAPS BAKED IN
  - esc[] is ALWAYS 8 long and slot 5 is the BRAKE, not a drive ESC. Samples are
    keyed on esc_address over the four drive addresses; timestamp == 0 entries
    are dropped. Never index esc[] by position.
  - erpm_to_ms 0.003900 is tape-validated but SPEED-DEPENDENT (over-reads ~5% at
    0.9 m/s, under-reads ~22% at crawl). Fit the ladder over the HIGH points,
    where the error is small, and confirm one floor point on tape.
  - COM_DISARM_PRFLT auto-disarms an idle armed rover after 10 s. Expect to
    re-arm between rungs; ⛔ do not change that param without asking.

USAGE
  python3 tools/throttle_ladder_record.py --out ladder.json          # record
  python3 tools/throttle_ladder_record.py --analyse ladder.json      # score

  Recording prints a live line per rung. Drive the stick to a level, HOLD it
  still for at least 2 s, return to centre, then step up. Five rungs is plenty.
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (EscStatus, InputRc, ManualControlSetpoint,
                          RoverThrottleSetpoint, VehicleStatus)
from nav_msgs.msg import Odometry

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

DRIVE = (10, 11, 12, 13)          # the four drive ESCs, BY ADDRESS
LEFT, RIGHT = {11, 13}, {10, 12}

# 🔴 ERPM SIGNS ARE A LIVE DISPUTE — THIS FILE RECORDS RAW AND DECIDES LATER.
# PRODUCTION is what wheel_odometry_node applies today (its `wheel_signs`
# parameter). MEASURED is what the wheels were doing on 2026-09-12: at driven
# full forward throttle all four ESCs reported POSITIVE raw ERPM, and the
# operator confirmed by eye that all four wheels turn forward together. Under
# the PRODUCTION map the right side computes as (-1487 + 1532)/2 ~ 9 ERPM
# instead of ~1500, i.e. it cancels itself and /odom reads about half.
# ⛔ Do not silently pick one. Record raw, report both, and let a reverse burst
# settle it: in reverse all four raw values should flip together.
PRODUCTION_SIGNS = {10: -1.0, 11: 1.0, 12: 1.0, 13: 1.0}
MEASURED_SIGNS = {10: 1.0, 11: 1.0, 12: 1.0, 13: 1.0}
ERPM_TO_MS = 0.003900             # MUST match wheel_odometry_node
NAV = {0: 'Manual', 4: 'Hold', 23: 'AutoNav'}

STEADY_TOL = 0.02                 # throttle counts as held within +/- this
MIN_HOLD = 1.0                    # [s] a rung must be held at least this long
SETTLED_FRAC = 0.5                # score only the last half of a rung


class Recorder(Node):
    def __init__(self):
        super().__init__('throttle_ladder_record')
        self.samples = []
        self.throttle = None      # what PX4 published (only while the mode runs)
        self.stick = None         # normalized stick, the DISARMED fallback
        self.nav = self.arm = None
        self.online = 0
        self.ch = []
        self.odom_vx = None
        self.create_subscription(RoverThrottleSetpoint, '/fmu/out/rover_throttle_setpoint',
                                 self.thr, PX4_QOS)
        self.create_subscription(ManualControlSetpoint, '/fmu/out/manual_control_setpoint',
                                 self.man, PX4_QOS)
        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc, PX4_QOS)
        self.create_subscription(InputRc, '/fmu/out/input_rc', self.rc, PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.st, PX4_QOS)
        self.create_subscription(Odometry, '/odom', self.odom, 10)

    def thr(self, m):
        self.throttle = float(m.throttle_body_x)

    def man(self, m):
        # In Manual this IS throttle_body_x (DifferentialManualMode::manual copies
        # it straight through), and unlike rover_throttle_setpoint it publishes
        # while disarmed -- so the ladder can be rehearsed before arming.
        self.stick = float(m.throttle) if m.valid else None

    def rc(self, m):
        self.ch = [int(v) for v in m.values[:8]]

    def st(self, m):
        self.nav, self.arm = int(m.nav_state), int(m.arming_state)

    def odom(self, m):
        self.odom_vx = float(m.twist.twist.linear.x)

    def esc(self, m):
        self.online = int(m.esc_online_flags)
        rpm, cur = {}, {}
        for i in range(len(m.esc)):
            e = m.esc[i]
            a = int(e.esc_address)
            if a in DRIVE and e.timestamp != 0:
                rpm[a] = float(e.esc_rpm)      # RAW -- no sign map applied here
                cur[a] = float(e.esc_current)
        if not rpm:
            return
        self.samples.append({
            't': time.time(),
            'throttle': self.throttle if self.throttle is not None else self.stick,
            'throttle_src': 'fc' if self.throttle is not None else 'stick',
            'stick': self.stick,
            'rpm': rpm,
            'current': cur,
            'online_flags': self.online,
            'nav': self.nav,
            'arm': self.arm,
            'odom_vx': self.odom_vx,
            'ch': list(self.ch),
        })


def lin_speed(rpm, signs=MEASURED_SIGNS):
    """Mean wheel speed [m/s] from a RAW per-address ERPM dict, under `signs`.

    None if either side is dark -- a one-sided average is not a speed.
    """
    lv = [v * signs.get(a, 1.0) for a, v in rpm.items() if a in LEFT]
    rv = [v * signs.get(a, 1.0) for a, v in rpm.items() if a in RIGHT]
    if not lv or not rv:
        return None
    return (sum(lv) / len(lv) + sum(rv) / len(rv)) / 2.0 * ERPM_TO_MS


def record(a):
    rclpy.init()
    n = Recorder()
    t0 = time.time()
    print('recording -- drive the stick to a level, HOLD 2 s, return to centre, step up.')
    print('⛔ hand on the kill switch (ch12).  Ctrl-C to stop early.\n')
    print(f'{"t":>7} {"nav":8} {"arm":6} {"thr":>6} {"src":>5} {"online":>7} '
          f'{"10":>6} {"11":>6} {"12":>6} {"13":>6} {"m/s":>7}')
    last = 0.0
    try:
        while time.time() - t0 < a.duration:
            rclpy.spin_once(n, timeout_sec=0.1)
            if not n.samples or time.time() - last < 0.5:
                continue
            last = time.time()
            s = n.samples[-1]
            v = lin_speed(s['rpm'])
            thr = '  none' if s['throttle'] is None else f"{s['throttle']:+6.3f}"
            print(f"{s['t']-t0:7.1f} {NAV.get(s['nav'], str(s['nav'])):8} "
                  f"{'ARMED ' if s['arm'] == 2 else 'disarm':6} {thr} "
                  f"{s['throttle_src']:>5} {s['online_flags']:7d} "
                  + ' '.join(f"{s['rpm'].get(x, float('nan')):6.0f}" for x in DRIVE)
                  + (f" {v:7.3f}" if v is not None else '     ---'), flush=True)
    except KeyboardInterrupt:
        pass

    out = a.out or f'ladder_{time.strftime("%Y%m%d_%H%M%S")}.json'
    with open(out, 'w') as f:
        json.dump({'erpm_to_ms': ERPM_TO_MS,
                   'rpm_is_raw': True,      # absent => an older file with the
                                            # production sign map baked in
                   'production_signs': PRODUCTION_SIGNS,
                   'samples': n.samples}, f)
    print(f'\n{len(n.samples)} samples -> {out}')
    print(f'score it: python3 tools/throttle_ladder_record.py --analyse {out}')
    n.destroy_node()
    rclpy.shutdown()


def segments(samples):
    """Split into rungs of steady, non-zero throttle with all four wheels awake."""
    segs, cur = [], []
    for s in samples:
        thr = s['throttle']
        ok = (thr is not None and abs(thr) > 0.01 and len(s['rpm']) == len(DRIVE))
        if ok and cur and abs(thr - cur[0]['throttle']) <= STEADY_TOL:
            cur.append(s)
        else:
            if len(cur) > 1 and cur[-1]['t'] - cur[0]['t'] >= MIN_HOLD:
                segs.append(cur)
            cur = [s] if ok else []
    if len(cur) > 1 and cur[-1]['t'] - cur[0]['t'] >= MIN_HOLD:
        segs.append(cur)
    return segs


def fit(points):
    """Least squares speed = k*throttle + c over (throttle, speed) points."""
    n = len(points)
    if n < 2:
        return None, None
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sxx = sum(p[0] * p[0] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None, None
    k = (n * sxy - sx * sy) / den
    c = (sy - k * sx) / n
    return k, c


def sign_audit(samples):
    """Report the RAW sign each address shows while actually driving.

    Coasting is worthless for this -- a sensorless ESC has no direction
    information when it is not driving, and on 2026-09-12 the reported signs
    scattered the moment the stick was centred. So only samples with real
    throttle and real rotation are counted.
    """
    fwd = {a: [0, 0] for a in DRIVE}     # [negative count, positive count]
    rev = {a: [0, 0] for a in DRIVE}
    for s in samples:
        thr = s['throttle']
        if thr is None or abs(thr) < 0.05:
            continue
        book = fwd if thr > 0 else rev
        for a, r in s['rpm'].items():
            if abs(r) > 50:
                book[a][1 if r > 0 else 0] += 1
    return fwd, rev


def analyse(path):
    d = json.load(open(path))
    samples = d['samples']

    if not d.get('rpm_is_raw'):
        # Older file: the production sign map was applied at record time. Undo it
        # so every path below works on raw ERPM.
        for s in samples:
            s['rpm'] = {int(a): v / PRODUCTION_SIGNS.get(int(a), 1.0)
                        for a, v in s['rpm'].items()}
        print('⚠️ old-format file: production signs un-applied to recover raw ERPM\n')
    else:
        for s in samples:
            s['rpm'] = {int(a): v for a, v in s['rpm'].items()}

    fwd, rev = sign_audit(samples)
    print('RAW ERPM SIGN WHILE DRIVING (counts, only |rpm| > 50 and |thr| > 0.05)')
    print(f'{"addr":>6} {"fwd -":>7} {"fwd +":>7} {"rev -":>7} {"rev +":>7}   reading')
    for a in DRIVE:
        f_neg, f_pos = fwd[a]
        r_neg, r_pos = rev[a]
        if f_pos + f_neg == 0:
            note = 'never driven forward'
        elif f_pos > f_neg and r_neg >= r_pos and (r_neg + r_pos) > 0:
            note = 'positive = forward  (sign +1)'
        elif f_neg > f_pos and r_pos >= r_neg and (r_neg + r_pos) > 0:
            note = 'negative = forward  (sign -1)'
        elif f_pos > f_neg:
            note = 'positive fwd; NO reverse sample to confirm'
        else:
            note = 'negative fwd; NO reverse sample to confirm'
        print(f'{a:>6} {f_neg:>7} {f_pos:>7} {r_neg:>7} {r_pos:>7}   {note}')
    print('🔑 A reverse burst is what makes this conclusive: every address should')
    print('   flip together. Without one, forward sign alone can be a wiring guess.')
    print(f'   wheel_odometry_node currently applies {PRODUCTION_SIGNS}\n')

    segs = segments(samples)
    if not segs:
        print('NO STEADY RUNGS FOUND.')
        print('  Either the throttle never held still for '
              f'{MIN_HOLD:.1f} s, or a wheel was asleep the whole time.')
        flags = {s['online_flags'] for s in samples}
        print(f'  esc_online_flags seen: {sorted(flags)} (15 = all four awake)')
        return

    src = {s.get('throttle_src') for s in samples if s['throttle'] is not None}
    print(f'{len(segs)} steady rungs, {len(samples)} samples, throttle from {sorted(src)}')
    if src == {'stick'}:
        print('⚠️ every throttle value came from the STICK, not from the FC output --')
        print('   fine in Manual (the firmware copies it straight through), but it')
        print('   means the rover was never armed, so the wheels cannot have turned.')
    print()
    print(f'{"thr":>7} {"hold":>6} {"m/s meas":>9} {"m/s prod":>9} {"ERPM avg":>9} '
          f'{"spread":>7} {"A avg":>7}  ratio m/s per unit thr')
    points = []
    skipped = 0
    for seg in segs:
        tail = seg[int(len(seg) * (1 - SETTLED_FRAC)):]
        thr = sum(s['throttle'] for s in tail) / len(tail)

        # A steady THROTTLE is not a steady SPEED. A rung recorded just after the
        # stick dropped is a spin-down: ERPM still falling, current NEGATIVE
        # (the motor is regenerating, not driving). Scoring those as rungs
        # invents low-throttle points that were never held.
        mags = [sum(abs(r) for r in s['rpm'].values()) / len(s['rpm']) for s in tail]
        mean_mag = sum(mags) / len(mags)
        drift = (max(mags) - min(mags)) / mean_mag if mean_mag > 1.0 else 0.0
        mean_amp = sum(c for s in tail for c in s['current'].values()) / \
            max(1, sum(len(s['current']) for s in tail))
        if mean_mag > 1.0 and (drift > 0.10 or mean_amp < 0.0):
            skipped += 1
            continue

        speeds = [lin_speed(s['rpm']) for s in tail]
        speeds = [v for v in speeds if v is not None]
        prod = [lin_speed(s['rpm'], PRODUCTION_SIGNS) for s in tail]
        prod = [v for v in prod if v is not None]
        if not speeds:
            continue
        v = sum(speeds) / len(speeds)
        v_prod = sum(prod) / len(prod) if prod else float('nan')
        erpms = [abs(r) for s in tail for r in s['rpm'].values()]
        amps = [c for s in tail for c in s['current'].values()]
        per_wheel = {a: sum(abs(s['rpm'][a]) for s in tail) / len(tail) for a in DRIVE}
        spread = (max(per_wheel.values()) - min(per_wheel.values())) / max(1.0, max(per_wheel.values()))
        print(f'{thr:+7.3f} {seg[-1]["t"]-seg[0]["t"]:5.1f}s {v:9.3f} {v_prod:9.3f} '
              f'{sum(erpms)/len(erpms):9.0f} {spread:6.1%} {sum(amps)/len(amps):7.2f}'
              f'  {v/thr if abs(thr) > 1e-6 else float("nan"):8.3f}')
        points.append((thr, v))

    k, c = fit(points)
    print()
    if skipped:
        print(f'({skipped} rungs dropped: ERPM still drifting >10% or current '
              f'negative -- spin-down, not a held level)')
    # Rungs below the friction threshold never moved; they say nothing about the
    # throttle-to-speed slope, so they are excluded from the flatness test.
    moving = [p for p in points if abs(p[1]) > 0.05]
    flat = (len(moving) > 2
            and (max(p[1] for p in moving) - min(p[1] for p in moving))
            / max(1e-6, max(p[1] for p in moving)) < 0.10
            and max(p[0] for p in moving) > 2 * min(p[0] for p in moving))
    if flat:
        print('🔴 SPEED IS FLAT ACROSS THE THROTTLE RANGE -- throttle is not')
        print('   setting speed here at all. That is the signature of a TORQUE')
        print('   (current) command with no load to push against: any command')
        print('   above friction accelerates to the same ceiling. A linear fit')
        print('   through these points is meaningless; do not read one off.')
        print('   ⇒ This test cannot be completed on stands. It needs the floor.')
        return
    if k is None:
        print('Not enough rungs to fit a line -- need at least two.')
        return
    deadband = -c / k if abs(k) > 1e-9 else float('nan')
    print(f'FIT: speed = {k:.3f} * throttle {c:+.3f}   (deadband at throttle {deadband:.3f})')
    print(f'  => speed at FULL throttle (1.0) = {k + c:.2f} m/s')
    print(f'  RO_MAX_THR_SPEED reads 0.60 today => ratio {(k + c) / 0.60:.2f}x')
    print()
    print('⛔ If this was a WHEELS-UP run, that figure is an UPPER BOUND (no load,')
    print('   no slip). It bounds the answer and maps the deadband; it does NOT')
    print('   set RO_MAX_THR_SPEED. A loaded floor point does that.')
    print('🔑 The per-rung ratio column should be FLAT if a wrong feedforward')
    print('   constant is the whole story. A ratio that climbs with throttle')
    print('   means something else is in the path too.')
    print('🔑 "m/s meas" vs "m/s prod" is the sign dispute in one column pair:')
    print('   prod is what /odom publishes today. If they differ by ~2x, /odom is')
    print('   under-reading by that much and the EKF bridge is feeding PX4 a')
    print('   velocity that is half the truth.')

    dropouts = [s for s in samples
                if s['throttle'] is not None and abs(s['throttle']) > 0.05
                and any(abs(r) < 1e-6 for r in s['rpm'].values())
                and any(abs(r) > 50 for r in s['rpm'].values())]
    if dropouts:
        lo = min(max(abs(r) for r in s['rpm'].values()) for s in dropouts)
        print(f'\n⚠️ ESC ZERO-DROPOUT: {len(dropouts)} samples where one wheel read exactly 0')
        print(f'   while another turned. Lowest co-occurring ERPM: {lo:.0f}'
              f' ({lo*ERPM_TO_MS:.3f} m/s). This is G2 item (a).')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=300.0,
                    help='recording length [s]')
    ap.add_argument('--out', default=None, help='JSON output path')
    ap.add_argument('--analyse', help='score a JSON from a previous run and exit')
    a = ap.parse_args()
    if a.analyse:
        analyse(a.analyse)
    else:
        record(a)


if __name__ == '__main__':
    main()
