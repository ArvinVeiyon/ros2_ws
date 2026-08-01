#!/usr/bin/env python3
"""Pre-flight check: does the collision-stop actually see what is in front of it?

Passive. Commands nothing, never arms. Park the rover where the run will start
and run this BEFORE arming.

It mirrors the reflex collision-stop in autonav_mode/mode.hpp exactly -- same
+/-20 deg sector, same validity test, same front_overhang subtraction, same
hysteresis thresholds -- so what it prints is what the executor will decide.

The number that matters is not just distance but COVERAGE: the fraction of rays
in the forward sector that return a valid range. A depth camera aimed at a
smooth wall at a grazing angle, or at glass, gloss or a very dark surface, can
return almost nothing -- and an empty sector reads as "infinity, nothing there",
which is indistinguishable from open floor. Low coverage means the collision
stop is running on hope, so it is called out loudly here rather than discovered
by driving into something.

Usage:
    python3 tools/preflight_scan_check.py [seconds]
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

# Mirrors autonav_mode/include/autonav_mode/mode.hpp. If those change, change these.
SECTOR_HALF = 0.35      # rad, +/- forward sector (~20 deg)
FRONT_OVERHANG = 0.337  # m, scan origin -> front bumper (MEASURED 2026-07-28)
STOP_DISTANCE = 0.35    # m, bumper clearance below which forward is blocked
CLEAR_DISTANCE = 0.50   # m, bumper clearance above which the block releases
SCAN_TIMEOUT = 0.5      # s


class Preflight(Node):

    def __init__(self):
        super().__init__('preflight_scan_check')
        # /scan is published both RELIABLE and BEST_EFFORT; BEST_EFFORT matches
        # the sensor pipeline and will not block on a slow subscriber.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, '/scan', self.cb, qos)
        self.mins = []          # per-scan sector minimum (raw range)
        self.coverage = []      # per-scan valid fraction within the sector
        self.spreads = []       # per-scan (max - min) across valid sector rays
        self.n_empty = 0        # scans with NO valid return in the sector
        self.n = 0
        self.sector_rays = 0

    def cb(self, scan: LaserScan):
        self.n += 1
        vals = []
        total = 0
        for i, r in enumerate(scan.ranges):
            ang = scan.angle_min + i * scan.angle_increment
            if ang < -SECTOR_HALF or ang > SECTOR_HALF:
                continue
            total += 1
            # Exactly the executor's validity test.
            if not math.isfinite(r) or r <= 0.0 or r < scan.range_min or r > scan.range_max:
                continue
            vals.append(r)
        self.sector_rays = total
        if total:
            self.coverage.append(len(vals) / total)
        if vals:
            self.mins.append(min(vals))
            self.spreads.append(max(vals) - min(vals))
        else:
            self.n_empty += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seconds', nargs='?', type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = Preflight()
    print(f'\nreading /scan for {args.seconds:.0f}s — hold the rover still...\n', flush=True)
    import time
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    print('=' * 64)
    if node.n == 0:
        print('NO /scan RECEIVED AT ALL.')
        print('  rover-scan / rover-camera not running, or the camera is down.')
        print('  ARMING NOW WOULD BLOCK FORWARD IMMEDIATELY (require_scan fail-safe).')
        print('=' * 64)
        rclpy.shutdown()
        return 1

    rate = node.n / args.seconds
    cov = sum(node.coverage) / len(node.coverage) if node.coverage else 0.0
    print(f'  scans received        : {node.n}  ({rate:.1f} Hz)')
    print(f'  rays in +/-20 sector  : {node.sector_rays}')
    print(f'  sector coverage       : {cov * 100:.1f}%  of rays return a valid range')
    print(f'  scans with EMPTY sector: {node.n_empty}')
    print()

    if not node.mins:
        print('  !! NOTHING VALID SEEN IN THE FORWARD SECTOR IN ANY SCAN.')
        print('     To the collision-stop this is identical to open floor —')
        print('     it will NOT stop you. Do not drive at anything on this reading.')
        print('     Re-aim at a matte, non-glossy surface, or move closer.')
        print('=' * 64)
        rclpy.shutdown()
        return 1

    raw_min = min(node.mins)
    raw_max = max(node.mins)
    raw_mean = sum(node.mins) / len(node.mins)
    jitter = raw_max - raw_min
    spread = sum(node.spreads) / len(node.spreads)
    clearance = raw_mean - FRONT_OVERHANG

    print(f'  raw sector min range  : {raw_mean:.3f} m'
          f'   (min {raw_min:.3f}, max {raw_max:.3f}, jitter {jitter:.3f})')
    print(f'  spread across sector  : {spread:.3f} m'
          f'   (near 0 = square to a flat surface)')
    print(f'  front_overhang        : {FRONT_OVERHANG:.3f} m')
    print(f'  => BUMPER CLEARANCE   : {clearance:.3f} m')
    print()
    print(f'  collision-stop blocks below {STOP_DISTANCE:.2f} m at the bumper'
          f'  (raw {STOP_DISTANCE + FRONT_OVERHANG:.2f} m)')
    print(f'  releases again above       {CLEAR_DISTANCE:.2f} m at the bumper'
          f'  (raw {CLEAR_DISTANCE + FRONT_OVERHANG:.2f} m)')
    print()

    runway = clearance - STOP_DISTANCE
    if clearance < STOP_DISTANCE:
        print(f'  >>> FORWARD IS BLOCKED RIGHT NOW ({clearance:.3f} < {STOP_DISTANCE:.2f}).')
        print('      AutoNav will refuse to drive forward from this spot.')
    else:
        print(f'  >>> forward is CLEAR. Usable runway before the stop fires:'
              f' {runway:.2f} m')
    print()

    problems = []
    if cov < 0.60:
        problems.append(
            f'Sector coverage is only {cov * 100:.0f}%. This is a poor depth target —\n'
            '     glossy, dark, glass, or too oblique. An empty sector reads as\n'
            '     "nothing there", so the collision-stop degrades silently.')
    if node.n_empty:
        problems.append(
            f'{node.n_empty} scan(s) had NO valid return in the sector at all.\n'
            '     During those the rover is effectively blind ahead.')
    if rate < 10.0:
        problems.append(
            f'/scan is only {rate:.1f} Hz. At 0.2 m/s that is '
            f'{0.2 / rate * 100:.0f} cm travelled between looks.\n'
            '     Check whether FPV video is streaming — it costs ~21% of the rate.')
    if jitter > 0.05:
        problems.append(
            f'Sector minimum is jittering {jitter:.3f} m between scans. The stop\n'
            '     distance will not be repeatable.')
    if spread > 0.15:
        problems.append(
            f'Spread across the sector is {spread:.3f} m — you are NOT square to a\n'
            '     flat surface. The minimum is off to one side of the cone, so the\n'
            '     stop will fire on a corner rather than on straight-ahead clearance.')

    if problems:
        print('  ISSUES:')
        for p in problems:
            print(f'   !! {p}')
    else:
        print('  No issues. Square to a flat surface, good coverage, stable range.')
    print('=' * 64)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
