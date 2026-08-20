#!/usr/bin/env python3
"""Per-wheel ERPM during a manual straight drive — is the crawl under-read the DEADBAND?

WHY THIS EXISTS
  `/odom` under-reads ~22% at crawl and over-reads ~+5% at ~0.9 m/s (three and
  one reproductions respectively, autonav_reference §5/§13). "Slip rising with
  speed" was the working story, but the crawl figure EXCEEDS the geometric
  0.004633, which slip cannot do. So something else is eating distance.

  RESOLVED 2026-08-13 -- AND THE PREMISE ABOVE WAS WRONG. 0.004633 was never a
  valid bound: BOTH of its inputs are mis-measured (103.34 -> 119.58 ERPM-s/rev
  pooled over six runs, and the wheel is 155 mm not 152.4). Corrected slip-free
  constant is 0.004058, and the configured 0.003900 then implies a physical 4.0%
  slip. THE SCALE IS INNOCENT: the ~20% crawl under-read is the ESC zero-dropout
  corrupting the integral. See autonav_reference §5/§10.

  wheel_odometry_node.py:389 zeroes EACH WHEEL under `deadband_erpm` (5.0) and
  then averages the side. At crawl the wheels sit near 36 ERPM, so a wheel that
  dips under 5 is counted as a STOPPED wheel and drags its side's mean down.
  That is an under-read which exists only at low speed and needs no slip at all.

  THE FALSIFIABLE PREDICTION: if deadband-zeroing is the cause, the fraction of
  per-wheel samples under 5 ERPM at crawl should be of the same order as the
  under-read (~20%). If it is ~0%, this hypothesis is DEAD and it really is slip.
  Tonight's speed run could not answer it because it logged only max(rpm) across
  the four wheels -- exactly the reduction that hides this.

WHAT IT DOES NOT DO
  Never arms. Never commands. Never publishes. Drive from the RC in Manual --
  open loop, no estimator and no yaw controller in the path, no EKF bridge
  needed. Read-only from start to finish.

THREE INTEGRALS, because the difference between them IS the deadband's cost
  raw   -- every reading, no deadband. The honest distance integral.
  node  -- per-wheel zeroing at 5.0 then side means: what /odom ACTUALLY does.
  comb  -- deadband applied to the COMBINED mean at 40.0, which is what
           tools/odom_scale_measure.py does. It is documented there as
           "matching wheel_odometry_node" and it does NOT: different threshold,
           applied at a different stage. Reported so the gap is visible.

COVERAGE IS THE THING TO WATCH. An earlier tool in this repo silently dropped
frames and under-counted a real 2 m drive by ~5x, which reads as a spectacular
scale error rather than a measurement bug. Every skipped frame and every second
of lost time is counted here and shown, and a poor-coverage run is refused.

Usage
    python3 tools/wheel_erpm_log.py                     # drive, Ctrl-C to end
    python3 tools/wheel_erpm_log.py --tape 2.430        # with the taped distance
    python3 tools/wheel_erpm_log.py --analyse FILE.json --tape 2.430

Run it 3-4 times across a SPREAD OF SPEEDS. One run gives one point on a curve;
`erpm_to_ms` is speed-dependent and a single "corrected" constant would break the
regime house_map_v4 was built in.
"""

import argparse
import json
import math
import os
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)

from px4_msgs.msg import EscStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

# addr 10 reports inverted ERPM (manual_drive_log.py, l2_floortest_wheel0_reversed)
SIGN = {10: -1.0, 11: 1.0, 12: 1.0, 13: 1.0}
LEFT_ADDRS, RIGHT_ADDRS = {11, 13}, {10, 12}

NODE_DEADBAND = 5.0     # wheel_odometry_node.py:121 -- PER WHEEL, before averaging
COMB_DEADBAND = 40.0    # odom_scale_measure.py:56   -- on the COMBINED mean
ERPM_TO_MS = 0.003900   # wheel_odometry_node.py:116 -- the value under suspicion

FRONT_OVERHANG = 0.337  # scan origin -> bumper, confirmed to 1 mm 2026-08-13
SECTOR_HALF = math.radians(20.0)   # the reflex's cone (safety)
NARROW_HALF = math.radians(5.0)    # the RULER's cone (measurement) -- see scan()
SCAN_SCALE = 1.0155     # /scan reads 1.55% short (autonav_reference §5)

MAX_DT = 0.2            # never integrate across a gap longer than this [s]
ESC_STALE_US = 200000


class WheelLog(Node):
    def __init__(self, settle, min_erpm):
        super().__init__('wheel_erpm_log')
        self.settle = settle
        self.min_erpm = min_erpm

        self.moving = False
        self.t_start = None
        self.t_quiet = None

        self.erpm_s = dict(raw=0.0, node=0.0, comb=0.0)
        self.counted_time = 0.0
        self.lost_time = 0.0
        self.n_gap = self.n_fallback = self.n_partial = 0
        self.prev_us = None

        # per wheel, only while moving
        self.w_n = defaultdict(int)
        self.w_sum = defaultdict(float)
        self.w_erpm_s = defaultdict(float)   # TIME-INTEGRATED per wheel: the
        #   quantity a hand-rotation calibration needs. ERPM-s / revolutions
        #   gives counts per revolution for THAT wheel, free of slip entirely.
        self.w_peak = defaultdict(float)
        self.w_under = defaultdict(int)     # |erpm| < NODE_DEADBAND
        self.w_zero = defaultdict(int)      # exactly 0
        self.w_missing = defaultdict(int)   # offline/stale while moving

        self.odom_dist = 0.0
        self.odom_prev = None
        self.odom_v = 0.0
        self.bumper = None
        self.bumper_first = None
        self.bumper_last = None

        self.samples = []

        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc, PX4_QOS)
        self.create_subscription(Odometry, '/odom', self.od, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        print('waiting for motion — drive the rover STRAIGHT from the RC in Manual')

    # ---------------------------------------------------------------- inputs
    def scan(self, m):
        # NOT the reflex's +/-20 deg MINIMUM. That is the right quantity for
        # collision safety and the WRONG one for measuring distance: any object
        # near the cone edge becomes the minimum and silently replaces the wall.
        # Measured 2026-08-13 -- with the rover stationary, edge objects dragged
        # the sector minimum from 2.27 m to 2.01 m while straight ahead was
        # unchanged. A narrow central median tracks the wall and ignores them.
        # +/-5 deg costs at most 1/cos(5) = +0.4%, and cancels in a DIFFERENCE.
        near = []
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = m.angle_min + i * m.angle_increment
            if abs(a) > NARROW_HALF:
                continue
            near.append(r)
        if near:
            near.sort()
            self.bumper = near[len(near) // 2] - FRONT_OVERHANG
        else:
            self.bumper = None

    def od(self, m):
        now = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom_v = m.twist.twist.linear.x
        if self.odom_prev is not None and self.moving:
            dt = now - self.odom_prev
            if 0.0 < dt < MAX_DT:
                self.odom_dist += abs(self.odom_v) * dt
        self.odom_prev = now

    def esc(self, msg: EscStatus):
        reports = list(msg.esc[:msg.esc_count])
        stamps = [r.timestamp for r in reports if r.timestamp > 0]
        if not stamps:
            return
        ref_us = max(stamps)

        wheels = {}
        for i, r in enumerate(reports):
            if r.timestamp == 0 or r.esc_address not in SIGN:
                continue
            if not (msg.esc_online_flags >> i) & 1:
                continue
            if ref_us - r.timestamp > ESC_STALE_US:
                continue
            wheels[r.esc_address] = float(r.esc_rpm) * SIGN[r.esc_address]

        left = [v for a, v in wheels.items() if a in LEFT_ADDRS]
        right = [v for a, v in wheels.items() if a in RIGHT_ADDRS]

        if left and right:
            raw = (sum(left) / len(left) + sum(right) / len(right)) / 2.0
        elif left or right:
            # One side dozing. Integrating the wheels we DO have beats deleting
            # real distance, but it is counted so the report can be judged.
            side = left or right
            raw = sum(side) / len(side)
            if self.moving:
                self.n_fallback += 1
        else:
            if self.moving:
                self.n_partial += 1
            return

        # THE NODE'S RULE: zero each wheel under the deadband, THEN average.
        nl = [0.0 if abs(v) < NODE_DEADBAND else v for v in left]
        nr = [0.0 if abs(v) < NODE_DEADBAND else v for v in right]
        if nl and nr:
            node_v = (sum(nl) / len(nl) + sum(nr) / len(nr)) / 2.0
        else:
            side = nl or nr
            node_v = (sum(side) / len(side)) if side else 0.0

        comb_v = 0.0 if abs(raw) < COMB_DEADBAND else raw

        if self.prev_us is not None:
            dt = (msg.timestamp - self.prev_us) / 1e6
            if dt <= 0.0:
                pass
            elif dt >= MAX_DT:
                if self.moving:
                    self.lost_time += dt
                    self.n_gap += 1
            else:
                # Start/stop on the FASTEST wheel, not the four-wheel mean: when
                # one wheel is spun by hand the mean stays near zero and a
                # mean-based trigger would never see the run at all.
                act = max((abs(v) for v in wheels.values()), default=0.0)
                if not self.moving and act > self.min_erpm:
                    print('  motion detected — logging...', flush=True)
                    self.moving = True
                    self.t_start = msg.timestamp
                    self.bumper_first = self.bumper
                if self.moving:
                    self.erpm_s['raw'] += raw * dt
                    self.erpm_s['node'] += node_v * dt
                    self.erpm_s['comb'] += comb_v * dt
                    self.counted_time += dt

                    for a in SIGN:
                        if a not in wheels:
                            self.w_missing[a] += 1
                            continue
                        v = abs(wheels[a])
                        self.w_n[a] += 1
                        self.w_sum[a] += v
                        self.w_erpm_s[a] += v * dt
                        self.w_peak[a] = max(self.w_peak[a], v)
                        if v < NODE_DEADBAND:
                            self.w_under[a] += 1
                        if v == 0.0:
                            self.w_zero[a] += 1

                    if self.bumper is not None:
                        self.bumper_last = self.bumper
                    self.samples.append(dict(
                        t=round((msg.timestamp - self.t_start) / 1e6, 4),
                        w={str(a): wheels.get(a) for a in SIGN},
                        raw=round(raw, 2), node=round(node_v, 2),
                        odom_v=round(self.odom_v, 4),
                        bumper=(round(self.bumper, 4) if self.bumper is not None else None)))

                    if act >= NODE_DEADBAND:
                        self.t_quiet = None
                    elif self.t_quiet is None:
                        self.t_quiet = time.time()
        self.prev_us = msg.timestamp

    def done(self):
        return (self.moving and self.t_quiet is not None
                and time.time() - self.t_quiet > self.settle)

    def dump(self):
        return dict(
            erpm_s=self.erpm_s, counted_time=self.counted_time,
            lost_time=self.lost_time, n_gap=self.n_gap,
            n_fallback=self.n_fallback, n_partial=self.n_partial,
            odom_dist=self.odom_dist,
            wheel_erpm_s={str(a): self.w_erpm_s[a] for a in SIGN},
            bumper_first=self.bumper_first, bumper_last=self.bumper_last,
            wheels={str(a): dict(n=self.w_n[a], mean=(self.w_sum[a] / self.w_n[a]
                                                      if self.w_n[a] else 0.0),
                                 peak=self.w_peak[a], under=self.w_under[a],
                                 zero=self.w_zero[a], missing=self.w_missing[a])
                    for a in SIGN},
            samples=self.samples)


# --------------------------------------------------------------------- report
def report(d, tape=None, revs=None, wheel_d=0.1524):
    ct, lt = d['counted_time'], d['lost_time']
    total = ct + lt
    cov = 100.0 * ct / total if total > 0 else 0.0

    print('\n' + '=' * 66)
    print(f"  counted {ct:.2f} s   lost {lt:.2f} s   coverage {cov:.1f}%")
    print(f"  gaps {d['n_gap']}   one-side frames {d['n_fallback']}   dropped {d['n_partial']}")
    if cov < 95.0:
        print('  !! COVERAGE TOO POOR — the integral is missing real motion.')
        print('     Do NOT read a scale factor off this run.')
        return

    raw, node, comb = d['erpm_s']['raw'], d['erpm_s']['node'], d['erpm_s']['comb']
    print(f"\n  ERPM-seconds   raw {raw:10.1f}   node-rule {node:10.1f}   combined-rule {comb:10.1f}")
    if abs(raw) > 1e-9:
        eaten_n = 100.0 * (raw - node) / raw
        eaten_c = 100.0 * (raw - comb) / raw
        print(f"  deadband eats  node-rule (per-wheel @{NODE_DEADBAND:.0f}) {eaten_n:+6.2f}%"
              f"   combined (@{COMB_DEADBAND:.0f}) {eaten_c:+6.2f}%")

    # rulers
    bf, bl = d.get('bumper_first'), d.get('bumper_last')
    bump = (bf - bl) * SCAN_SCALE if (bf is not None and bl is not None) else None
    print(f"\n  /odom integrated travel   {d['odom_dist']:.3f} m")
    if bump is not None:
        print(f"  bumper travel (independent) {bump:+.3f} m"
              f"   [{bf:.3f} -> {bl:.3f}, 1.55% corrected]")
        if bump > 0.05:
            print(f"    /odom vs bumper: {100*(d['odom_dist']/bump - 1):+.1f}%")
    if tape:
        print(f"  tape                      {tape:.3f} m")
        for k, v in (('raw', raw), ('node-rule', node), ('combined', comb)):
            if abs(v) > 1e-6:
                print(f"    erpm_to_ms from {k:9s} = {tape/abs(v):.6f}")
        print(f"    (configured {ERPM_TO_MS:.6f} ground-distance; "
              f"slip-free 0.004058 => 4.0% slip. 0.004633 is REFUTED)")

    # ------------------------------------------------- revolutions, if counted
    # Counting revolutions splits the constant into its two independent halves:
    #   ERPM-s / rev  = what the ENCODER counts per turn   (no ground involved)
    #   metres  / rev = the EFFECTIVE ROLLING CIRCUMFERENCE (no encoder involved)
    # ANSWERED 2026-08-13: BOTH numbers were wrong, in the same direction.
    #   ERPM-s/rev  103.34 -> 119.58 (pooled over SIX runs, sd 9.9%)  => R = 2.00
    #   wheel        152.4 -> 155.0 mm (measured; 6" nominal runs OVER nominal)
    # R = 2 because si_motor_poles holds the pole-PAIR count where the pole COUNT
    # belongs; the firmware already divides pole pairs out (canard_driver.c:494),
    # and these are direct-drive hub motors, so R would otherwise be 1.
    # WARNING: the ONE-RUN error here is the HUMAN REVOLUTION COUNT, +/-10-16%.
    # Never trust a single run -- two runs on the SAME wheel at a nominal 30 revs
    # disagreed by 16% on 2026-08-13.
    if revs:
        print(f"\n  REVOLUTIONS COUNTED: {revs:g}")
        ws = d.get('wheel_erpm_s', {})
        for a in sorted(SIGN, key=int):
            e = ws.get(str(a), 0.0)
            if e <= 1e-6:
                continue
            per_rev = e / revs
            print(f"    addr {a}: {e:9.1f} ERPM-s -> {per_rev:8.2f} ERPM-s/rev"
                  f"   (pooled 2026-08-13: 119.58 => R=2.00)")
        if tape:
            m_per_rev = tape / revs
            geo = math.pi * wheel_d
            print(f"\n    metres per revolution  {m_per_rev:.4f} m"
                  f"   vs geometric pi*{wheel_d} = {geo:.4f} m   ({100*(m_per_rev/geo-1):+.1f}%)")
            print(f"    => effective rolling diameter {m_per_rev/math.pi:.4f} m")
            if m_per_rev < geo * 0.999:
                print('    Driven: this shortfall is SLIP + tyre compression, measured')
                print('    directly for the first time. Hand-PUSHED, slip is ~0, so the')
                print('    same run pushed gives the true circumference -- and the gap')
                print('    between pushed and driven IS the slip.')
            else:
                print('    !! Rolling circumference EXCEEDS the geometric value. Slip')
                print('       cannot do that -- the 0.1524 m diameter is too small, or')
                print('       the revolution count is off. Re-measure the wheel.')

    print(f"\n  PER WHEEL (while moving)     mean ERPM   peak   <{NODE_DEADBAND:.0f} ERPM   ==0    missing")
    unders = []
    for a in sorted(SIGN, key=int):
        w = d['wheels'][str(a)]
        n = w['n']
        if not n:
            print(f"    addr {a}: NO DATA")
            continue
        side = 'L' if int(a) in LEFT_ADDRS else 'R'
        pu = 100.0 * w['under'] / n
        pz = 100.0 * w['zero'] / n
        unders.append(pu)
        print(f"    addr {a} ({side}):  {w['mean']:9.1f}  {w['peak']:6.0f}"
              f"   {pu:6.1f}%  {pz:5.1f}%   {w['missing']:5d}")

    means = {int(a): d['wheels'][a]['mean'] for a in d['wheels'] if d['wheels'][a]['n']}
    if means:
        lm = [v for a, v in means.items() if a in LEFT_ADDRS]
        rm = [v for a, v in means.items() if a in RIGHT_ADDRS]
        if lm and rm:
            L, R = sum(lm) / len(lm), sum(rm) / len(rm)
            if max(L, R) > 0:
                print(f"\n  side means   L {L:.1f}   R {R:.1f}"
                      f"   asymmetry {100*abs(L-R)/max(L, R):.1f}%"
                      "   (a straight run should be small)")
        sp = max(means.values()) - min(means.values())
        if max(means.values()) > 0 and sp / max(means.values()) > 0.15:
            print(f"  !! wheel spread {100*sp/max(means.values()):.0f}% — one wheel is not"
                  " turning like the others. Check before trusting the scale.")

    # ------------------------------------------------------------- hypothesis
    print('\n  DEADBAND HYPOTHESIS')
    mean_all = sum(means.values()) / len(means) if means else 0.0
    worst = max(unders) if unders else 0.0
    print(f"    mean wheel ERPM {mean_all:.1f}  (~{mean_all*ERPM_TO_MS:.3f} m/s at the configured constant)")
    if mean_all > 8 * NODE_DEADBAND and worst < 1.0:
        print('    => NOT THE DEADBAND, at this speed. Wheels sit far above it and')
        print('       essentially never dip under. If /odom still under-reads here,')
        print('       the cause is elsewhere (slip, or the constant itself).')
    elif worst >= 5.0:
        lost = 100 * (raw - node) / raw if abs(raw) > 1e-9 else 0.0
        print(f"    => Up to {worst:.1f}% of one wheel's samples read under the deadband,")
        print(f"       and the node-rule integral loses {lost:+.2f}% of the distance.")
        print(f"       ⚠️ READ THAT SECOND NUMBER, NOT THE FIRST. Zeroing a wheel that")
        print(f"       reads 2 ERPM removes 2, not {mean_all:.0f} -- the deadband can only ever")
        print(f"       delete what is BELOW {NODE_DEADBAND:.0f} ERPM, so it is arithmetically")
        print( "       incapable of a 20%+ under-read on wheels averaging well above it.")
        print( "       If /odom is short by far more than the loss above, the cause is")
        print( "       NOT the deadband -- look at the ==0 and missing columns, where a")
        print( "       wheel that is genuinely turning gets reported as stopped.")
    else:
        print('    => INCONCLUSIVE at this speed. Run it slower, where the wheels')
        print('       actually approach the deadband, before ruling either way.')
    print('  NB one run is ONE POINT on a speed-dependent curve. Repeat at 3-4 speeds.')
    print('=' * 66)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tape', type=float, default=None,
                    help='taped straight-line distance for this run [m]')
    ap.add_argument('--settle', type=float, default=3.0,
                    help='end the run after this long below the deadband [s]')
    ap.add_argument('--min-erpm', type=float, default=15.0,
                    help='ERPM above which the run is considered started')
    ap.add_argument('--revs', type=float, default=None,
                    help='revolutions counted on the marked wheel during the run')
    ap.add_argument('--wheel-diameter', type=float, default=0.1524,
                    help='assumed wheel diameter [m] (0.1524 = 6 in)')
    ap.add_argument('--out', default=None, help='JSON output path')
    ap.add_argument('--analyse', help='re-score a previous run and exit')
    a = ap.parse_args()

    if a.revs is not None:
        # A hand-turned or crawling wheel never reaches the driving threshold;
        # keeping it would discard the entire run (odom_scale_measure.py:366).
        a.min_erpm = min(a.min_erpm, 3.0)

    if a.analyse:
        report(json.load(open(a.analyse)), a.tape, a.revs, a.wheel_diameter)
        return 0

    rclpy.init()
    n = WheelLog(a.settle, a.min_erpm)
    try:
        while rclpy.ok() and not n.done():
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        print('\n  interrupted — reporting what was captured')
    d = n.dump()
    path = a.out or os.path.expanduser(
        time.strftime('~/wheel_erpm_%Y%m%d_%H%M%S.json'))
    with open(path, 'w') as f:
        json.dump(d, f)
    print(f'\n  log: {path}   ({len(d["samples"])} frames)')
    report(d, a.tape, a.revs, a.wheel_diameter)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
