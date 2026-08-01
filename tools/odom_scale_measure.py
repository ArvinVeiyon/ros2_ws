#!/usr/bin/env python3
"""Measure erpm_to_ms against a tape-measured straight drive.

Drive the rover a measured straight distance (RC / Manual is ideal -- open loop,
no estimator or yaw-rate controller in the path) and this reports the scale
factor directly:

    erpm_to_ms = tape_distance_metres / integrated_ERPM_seconds

The ERPM integral is computed here from raw /fmu/out/esc_status, so the result
does NOT depend on the erpm_to_ms currently in the config -- which is the whole
point, since that value is the thing under suspicion.

COVERAGE IS THE THING TO WATCH. The integral is only valid if it accounts for
essentially all of the time the rover was moving. A first version of this tool
silently dropped frames whose opposite side had gone missing, and gaps longer
than the dt sanity limit, and under-counted a real 2 m drive by ~5x -- which
would have been read as a spectacular scale error rather than a measurement bug.
So every skipped frame and every second of lost time is now counted and shown,
and the result is refused outright if coverage is poor.

Two integrals are reported:
  * raw       -- every ERPM reading, the honest distance integral
  * deadbanded-- readings under DEADBAND zeroed, matching wheel_odometry_node
They should agree closely on a brisk drive. A large gap between them means the
run was too slow and the node's deadband is eating real motion.

Usage:
    python3 tools/odom_scale_measure.py

Drive, then type the taped distance when it asks. Pass --distance only if you
already know it exactly.

A run auto-starts when the wheels begin turning and auto-ends after the rover
has been stopped for --settle seconds. Ctrl-C also ends it.
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import EscStatus
from nav_msgs.msg import Odometry

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=5)

# Must match config/rover_odometry.yaml.
LEFT_ADDRS = {11, 13}
RIGHT_ADDRS = {10, 12}
SIGN = {10: -1.0, 11: 1.0, 12: 1.0, 13: 1.0}
DEADBAND = 40.0
CURRENT_SCALE = 0.000380
ESC_STALE_US = 300000
MAX_DT = 0.5          # s, dt above this is treated as a gap, not a step


class ScaleMeasure(Node):

    def __init__(self, tape, settle, min_erpm, deadband=DEADBAND, revs=None,
                 wheel_d=0.1524):
        super().__init__('odom_scale_measure')
        self.tape = tape
        self.settle = settle
        self.min_erpm = min_erpm
        self.deadband = deadband
        # Revolution mode: distance comes from counted wheel turns and the wheel
        # diameter, so neither tape error nor wheel slip enters the result.
        self.revs = revs
        self.wheel_d = wheel_d
        if revs is not None:
            self.tape = revs * math.pi * wheel_d

        self.erpm_s_raw = 0.0       # integral of mean raw signed ERPM
        self.erpm_s_dead = 0.0      # same, with the node's deadband applied
        self.prev_us = None
        self.moving = False
        self.stopped_for = 0.0
        self.samples = 0
        self.peak_erpm = 0.0

        # Coverage bookkeeping -- the reason to trust or reject a run.
        self.t_first = None
        self.t_last = None
        self.counted_time = 0.0
        self.rolling_time = 0.0
        self.lost_time = 0.0
        self.n_partial = 0          # a side had no usable ESC report
        self.n_fallback = 0         # integrated from one side only
        self.n_gap = 0              # dt exceeded MAX_DT

        # /odom cross-check. RELIABLE -- a BEST_EFFORT subscriber reads nothing.
        self.odom_dist = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        # Pose at the START of this run. /odom's pose is absolute since the
        # odometry node booted, so straightness must be measured against where
        # this run began -- not against the node's origin, which includes every
        # earlier drive and made the first straightness figures meaningless.
        self.odom_x0 = 0.0
        self.odom_y0 = 0.0
        self.odom_prev = None
        self.max_yaw = 0.0

        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc, BE)
        self.create_subscription(Odometry, '/odom', self.od, 10)

        if tape is not None:
            print(f'\nTape distance to compare against: {tape:.3f} m', flush=True)
        else:
            print('\nDrive first — you will be asked for the taped distance after.',
                  flush=True)
        print('Drive the rover STRAIGHT forward on RC. Measuring starts by itself.\n',
              flush=True)

    def od(self, m):
        now = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.odom_prev is not None and self.moving:
            dt = now - self.odom_prev
            if 0.0 < dt < MAX_DT:
                self.odom_dist += abs(m.twist.twist.linear.x) * dt
        self.odom_prev = now
        self.odom_x = m.pose.pose.position.x
        self.odom_y = m.pose.pose.position.y
        self.max_yaw = max(self.max_yaw, abs(m.twist.twist.angular.z))

    def esc(self, msg: EscStatus):
        reports = list(msg.esc[:msg.esc_count])
        stamps = [r.timestamp for r in reports if r.timestamp > 0]
        if not stamps:
            return
        ref_us = max(stamps)

        left, right = [], []
        for i, r in enumerate(reports):
            if r.timestamp == 0 or r.esc_address not in SIGN:
                continue
            if not (msg.esc_online_flags >> i) & 1:
                continue
            if ref_us - r.timestamp > ESC_STALE_US:
                continue
            e = float(r.esc_rpm) * SIGN[r.esc_address]
            (left if r.esc_address in LEFT_ADDRS else right).append(e)

        if left and right:
            raw = (sum(left) / len(left) + sum(right) / len(right)) / 2.0
        elif left or right:
            # One side dozing or dropped. Rather than discard the frame -- which
            # silently deletes real distance from the integral -- integrate the
            # wheels we do have. On a straight run both sides turn alike, so this
            # is a good approximation, and it is counted so it can be judged.
            side = left or right
            raw = sum(side) / len(side)
            if self.moving:
                self.n_fallback += 1
        else:
            if self.moving:
                self.n_partial += 1
            return

        dead = 0.0 if abs(raw) < self.deadband else raw
        self.peak_erpm = max(self.peak_erpm, abs(raw))

        if self.prev_us is not None:
            dt = (msg.timestamp - self.prev_us) / 1e6
            over = abs(raw) > self.min_erpm
            rolling = abs(raw) >= self.deadband
            if dt <= 0.0:
                pass
            elif dt >= MAX_DT:
                # Never integrate across a long gap -- but do record that the
                # time existed, so coverage exposes it instead of hiding it.
                if self.moving:
                    self.lost_time += dt
                    self.n_gap += 1
            else:
                if not self.moving and over:
                    print('  motion detected — measuring...', flush=True)
                    self.moving = True
                    self.t_first = msg.timestamp
                    self.odom_x0, self.odom_y0 = self.odom_x, self.odom_y
                if self.moving:
                    # Integrate EVERY frame of the run, not only the brisk ones.
                    # Restricting integration to frames above min_erpm silently
                    # deleted the slow ramps at each end -- real distance that
                    # the tape still measures -- and inflated the scale. The
                    # deadband alone decides what counts as motion, so idle
                    # jitter contributes nothing while creep is kept.
                    self.erpm_s_raw += raw * dt
                    self.erpm_s_dead += dead * dt
                    self.counted_time += dt
                    self.samples += 1
                    if rolling:
                        self.t_last = msg.timestamp
                        self.rolling_time += dt
                    if self.samples % 100 == 0:
                        print(f'    ERPM·s={self.erpm_s_raw:9.1f}   '
                              f'rolling {self.rolling_time:5.1f} s', flush=True)
                    if over:
                        self.stopped_for = 0.0
                    else:
                        self.stopped_for += dt
                        if self.stopped_for >= self.settle:
                            self.report()
                            raise SystemExit(0)
        self.prev_us = msg.timestamp

    def report(self):
        print('\n' + '=' * 64, flush=True)
        if abs(self.erpm_s_raw) < 1.0:
            print('No usable motion recorded — nothing to measure.', flush=True)
            print('=' * 64, flush=True)
            return

        duration = 0.0
        if self.t_first is not None and self.t_last is not None:
            duration = (self.t_last - self.t_first) / 1e6
        coverage = (self.counted_time / duration * 100.0) if duration > 0 else 0.0
        net = math.hypot(self.odom_x - self.odom_x0, self.odom_y - self.odom_y0)
        straightness = (net / self.odom_dist) if self.odom_dist > 0 else 0.0

        print(f'  integrated ERPM·s (raw) : {self.erpm_s_raw:9.1f}', flush=True)
        print(f'  integrated ERPM·s (dead): {self.erpm_s_dead:9.1f}', flush=True)
        print(f'  peak mean ERPM          : {self.peak_erpm:9.1f}', flush=True)
        print(f'  samples                 : {self.samples:9d}', flush=True)
        print(flush=True)
        print(f'  run duration            : {duration:9.2f} s', flush=True)
        print(f'  time actually counted   : {self.counted_time:9.2f} s'
              f'   ({coverage:.1f}% coverage)', flush=True)
        print(f'  of that, wheels rolling : {self.rolling_time:9.2f} s', flush=True)
        print(f'  time lost to gaps       : {self.lost_time:9.2f} s'
              f'   ({self.n_gap} gaps)', flush=True)
        print(f'  frames one-side-only    : {self.n_fallback:9d}', flush=True)
        print(f'  frames with no ESC data : {self.n_partial:9d}', flush=True)
        print(flush=True)
        print(f'  /odom path length       : {self.odom_dist:9.3f} m  (current scale)',
              flush=True)
        print(f'  /odom net displacement  : {net:9.3f} m', flush=True)
        print(f'  straightness (net/path) : {straightness:9.3f}   (1.0 = dead straight)',
              flush=True)
        print(f'  peak |yaw rate|         : {self.max_yaw:9.3f} rad/s', flush=True)
        print(flush=True)

        # Warn BEFORE asking for a distance, so problems are visible while the
        # run is still fresh and can be repeated on the spot.
        bad = False
        if coverage < 95.0:
            print(f'!! COVERAGE {coverage:.1f}% — the integral is missing real motion.',
                  flush=True)
            print('   The scale from this run would be WRONG HIGH. Redo it.', flush=True)
            bad = True
        if self.n_partial:
            print(f'!! {self.n_partial} frames had no usable ESC data at all.', flush=True)
            bad = True
        if straightness and straightness < 0.97:
            print(f'!! Path curved (straightness {straightness:.2f}). A straight-line',
                  flush=True)
            print('   tape reading understates the distance the wheels rolled,',
                  flush=True)
            print('   which biases the scale LOW. Redo it straighter.', flush=True)
            bad = True
        if self.peak_erpm < 100:
            print('!! Very low speed — timing error dominates. Drive faster.', flush=True)
            bad = True
        if abs(self.erpm_s_raw) > 0 and \
                abs(self.erpm_s_dead / self.erpm_s_raw - 1.0) > 0.02:
            pct = (1.0 - self.erpm_s_dead / self.erpm_s_raw) * 100.0
            print(f'!! Deadband is eating {pct:.1f}% of the motion — run was too slow.',
                  flush=True)
        if bad:
            print(flush=True)

        if self.tape is None:
            # Drive first, then tape what it actually travelled. Asking up front
            # invites rounding the measurement to the number already typed in.
            print('  Now tape the distance the rover actually travelled.', flush=True)
            while self.tape is None:
                try:
                    raw = input('  distance in metres (blank to skip): ').strip()
                except (EOFError, OSError):
                    raw = ''
                if not raw:
                    print('\n  No distance given. Compute it from the raw integral:'
                          f'\n      erpm_to_ms = tape / {self.erpm_s_raw:.1f}',
                          flush=True)
                    print('=' * 64, flush=True)
                    return
                try:
                    v = float(raw)
                    if v <= 0:
                        raise ValueError
                    self.tape = v
                except ValueError:
                    print('  need a positive number, e.g. 2.14', flush=True)
            print(flush=True)

        scale = self.tape / self.erpm_s_raw
        ratio = scale / CURRENT_SCALE
        full_stick = 1500.0 * scale

        if self.revs is not None:
            print(f'  wheel revolutions       : {self.revs:9.2f}', flush=True)
            print(f'  wheel circumference     : {math.pi * self.wheel_d:9.4f} m',
                  flush=True)
            print(f'  => rolled distance      : {self.tape:9.3f} m  (slip-free)',
                  flush=True)
        else:
            print(f'  tape distance (truth)   : {self.tape:9.3f} m', flush=True)
        print(f'  current erpm_to_ms      : {CURRENT_SCALE:.6f}', flush=True)
        print(f'  MEASURED erpm_to_ms     : {scale:.6f}   ({ratio:.2f}x current)',
              flush=True)
        print(flush=True)
        # The config derives the scale as pi*D / (pole_pairs * gear_ratio * 60),
        # so the measurement pins down that product directly. It should come out
        # very near a sensible motor configuration; a ragged value means the
        # measurement is still contaminated.
        product = math.pi * self.wheel_d / (scale * 60.0)
        print(f'  implies pole_pairs x gear_ratio = {product:.2f}'
              f'   (config assumes 21 = 7 x 3)', flush=True)
        nearest = round(product)
        if nearest >= 1:
            implied = math.pi * self.wheel_d / (nearest * 60.0)
            off = abs(product - nearest) / nearest * 100.0
            print(f'  nearest whole value {nearest} gives erpm_to_ms = {implied:.6f}'
                  f'  ({off:.1f}% away)', flush=True)
        print(flush=True)
        # Independent sanity check: full stick was measured at ~1500 ERPM and
        # ~0.58 m/s by hand. A scale that disagrees wildly with that is a
        # measurement artefact, not a discovery.
        print(f'  implies full stick (1500 ERPM) = {full_stick:.2f} m/s'
              f'  ({full_stick * 3.6:.1f} km/h)', flush=True)
        # NOTE: the often-quoted "~0.58 m/s at full stick" is NOT independent
        # evidence -- it was itself computed from the suspect 0.000380 scale, so
        # quoting it against a new measurement is circular. The only honest
        # cross-check is whether the implied speed matches what the rover is
        # observed to do, which is a judgement for the operator to make.
        print('  (compare against how fast the rover actually runs at full', flush=True)
        print('   stick — that is the only non-circular check available)', flush=True)
        print('=' * 64, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--distance', type=float, default=None,
                    help='taped distance in metres; omit to be asked after the drive '
                         '(preferred — measure what it did, not what you aimed for)')
    ap.add_argument('--settle', type=float, default=2.0,
                    help='seconds stopped before a run is considered finished')
    ap.add_argument('--min-erpm', type=float, default=45.0,
                    help='mean ERPM above which the rover counts as moving')
    ap.add_argument('--deadband', type=float, default=DEADBAND,
                    help='ERPM below which a reading is treated as noise; lower it '
                         'for a slow hand-pushed run')
    ap.add_argument('--revs', type=float, default=None,
                    help='wheel revolutions counted by hand. Distance is then '
                         'revs * pi * wheel diameter, which removes BOTH tape error '
                         'and wheel slip — the definitive measurement')
    ap.add_argument('--wheel-diameter', type=float, default=0.1524,
                    help='wheel diameter in metres (default 0.1524 = 6 inch)')
    args = ap.parse_args()

    if args.revs is not None:
        # A hand push turns the wheels slowly; the driving thresholds would
        # discard the whole run.
        args.deadband = min(args.deadband, 5.0)
        args.min_erpm = min(args.min_erpm, 8.0)

    rclpy.init()
    node = ScaleMeasure(args.distance, args.settle, args.min_erpm,
                        deadband=args.deadband, revs=args.revs,
                        wheel_d=args.wheel_diameter)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        if node.moving:
            node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
