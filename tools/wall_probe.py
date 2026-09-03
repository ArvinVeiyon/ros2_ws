#!/usr/bin/env python3
"""Absolute geometric reference: RANSAC-fit a line to /scan against a flat wall.

Passive. Subscribes to /scan and nothing else. Commands nothing, never arms,
does not touch the FC. Safe to run at any time, armed or disarmed.

WHY THIS TOOL EXISTS
    Every other ruler on this rover depends on something suspect. /odom depends
    on the wheels (and under-reads ~20% at crawl because of the ESC zero
    dropout). The FC's fused yaw is unusable indoors. A tape measure needs a
    human at the rover and cannot be run mid-test. This gives one measurement
    that depends on NONE of that: park square to a flat wall, and the geometry
    of the wall in the scan frame yields perpendicular distance and bearing
    directly. It is the rig that found the heading fault, the odometry scale
    error and the gyro validation (autonav_reference.md section 14).

    Documented in autonav_reference.md section 14 and setup_manual.md E4 since
    2026-08-14, but never actually written -- the file did not exist until
    2026-09-03, so anything that claimed to be blocked on "run wall_probe" was
    blocked on a tool that was not there.

WHAT IT REPORTS
    perpendicular distance  scan origin -> wall plane, along the wall normal.
                            NOT the minimum ray, which is noisier and lands on
                            whatever single beam happens to read shortest.
    bearing                 angle of the wall normal in the scan frame.
                            0 = square to the wall. Sign follows the scan frame:
                            positive = the wall normal points to the left of
                            straight ahead, i.e. the rover is yawed clockwise
                            (right) relative to square.
    fit RMS                 how well the returns actually form a line. A wall
                            should sit at a few mm. A large value means it is
                            not fitting a flat wall -- clutter, a corner, a
                            curtain, or a camera looking at the floor.
    coverage / inliers      perception health. A depth camera at a grazing
                            angle, or on glass/gloss/matt-black, returns almost
                            nothing, and an empty sector reads as "infinity,
                            nothing there" -- indistinguishable from open floor.

READ THIS BEFORE TRUSTING A NUMBER
    - Park SQUARE and CLOSE to a flat, featureless stretch of wall. A doorway,
      skirting board, a corner or furniture inside the sector will be fitted
      too, and RANSAC will happily return a confident answer for the wrong line.
    - /scan is known to read SHORT (see autonav_reference.md section 5). This
      tool reports what /scan says, uncorrected. Comparing it against tape is
      the point -- do not silently apply a scale factor here.
    - Start the measurement, then GO QUIET. Issuing commands during a
      perception measurement starves the camera and produces confident wrong
      answers (section 14).
    - The +/-1 mm, +/-0.26 deg figures in the manuals describe the spread of a
      good run, not an accuracy guarantee. This tool prints its own sd every
      time. Judge the run by that, not by the manual.

Usage:
    python3 tools/wall_probe.py                      # 40 scans, +/-20 deg sector
    python3 tools/wall_probe.py --scans 100          # tighter statistics
    python3 tools/wall_probe.py --sector-deg 30      # wider slice of wall
    python3 tools/wall_probe.py --overhang           # also print bumper clearance
    python3 tools/wall_probe.py --json               # machine-readable
"""

import argparse
import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

# Mirrors autonav_mode/include/autonav_mode/mode.hpp -- only used for --overhang.
FRONT_OVERHANG = 0.337  # m, scan origin -> front bumper (MEASURED 2026-07-28)
# 2026-08-16: the camera was physically rotated to reach the FC, so this value
# and everything else camera-referenced is SUSPECT until re-verified. That
# re-verification is what this tool is for.


def fit_line(points, tol, iters, rng_seed=12345):
    """RANSAC a line to 2D points, then refine on the inliers by total least squares.

    Returns (nx, ny, c, inliers, rms) for the line {p : n.p = c} with |n| = 1 and
    c > 0, so c is the perpendicular distance from the origin and n points from
    the origin towards the wall. Returns None if no consensus is found.
    """
    n_pts = len(points)
    if n_pts < 2:
        return None

    # Deterministic sampling: same scan in, same answer out. A measurement tool
    # that returns a slightly different number each run is hard to argue with.
    state = rng_seed
    def nxt(limit):
        nonlocal state
        state = (1103515245 * state + 12345) & 0x7FFFFFFF
        return state % limit

    best_inliers = []
    for _ in range(iters):
        i = nxt(n_pts)
        j = nxt(n_pts)
        if i == j:
            continue
        (x1, y1), (x2, y2) = points[i], points[j]
        dx, dy = x2 - x1, y2 - y1
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            continue
        nx, ny = -dy / norm, dx / norm          # unit normal
        c = nx * x1 + ny * y1
        inliers = [p for p in points if abs(nx * p[0] + ny * p[1] - c) <= tol]
        if len(inliers) > len(best_inliers):
            best_inliers = inliers

    if len(best_inliers) < 2:
        return None

    # Total least squares on the inliers: the normal is the eigenvector of the
    # scatter matrix with the smaller eigenvalue. Ordinary least squares would
    # bias the fit as the wall approaches vertical in the scan frame.
    mx = sum(p[0] for p in best_inliers) / len(best_inliers)
    my = sum(p[1] for p in best_inliers) / len(best_inliers)
    sxx = sum((p[0] - mx) ** 2 for p in best_inliers)
    syy = sum((p[1] - my) ** 2 for p in best_inliers)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in best_inliers)
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)   # direction of max variance
    nx, ny = -math.sin(theta), math.cos(theta)       # normal is perpendicular
    c = nx * mx + ny * my
    if c < 0:                                        # point the normal at the wall
        nx, ny, c = -nx, -ny, -c

    resid = [nx * p[0] + ny * p[1] - c for p in best_inliers]
    rms = math.sqrt(sum(r * r for r in resid) / len(resid))
    return nx, ny, c, best_inliers, rms


class WallProbe(Node):

    def __init__(self, topic, sector, tol, iters):
        super().__init__('wall_probe')
        # /scan is published both RELIABLE and BEST_EFFORT; BEST_EFFORT matches
        # the sensor pipeline and will not block on a slow subscriber.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, topic, self.cb, qos)
        self.sector = sector
        self.tol = tol
        self.iters = iters

        self.dists = []         # per-scan perpendicular distance, m
        self.bearings = []      # per-scan wall-normal bearing, deg
        self.rms = []           # per-scan fit residual, m
        self.inlier_frac = []   # inliers / points offered to the fit
        self.coverage = []      # valid returns / rays in sector
        self.n_scans = 0
        self.n_nofit = 0
        self.sector_rays = 0
        self.frame_id = ''
        self.angle_min = None
        self.angle_max = None
        self.n_rays_total = 0

    def cb(self, scan: LaserScan):
        self.n_scans += 1
        self.frame_id = scan.header.frame_id
        self.angle_min, self.angle_max = scan.angle_min, scan.angle_max
        self.n_rays_total = len(scan.ranges)

        pts = []
        total = 0
        for i, r in enumerate(scan.ranges):
            ang = scan.angle_min + i * scan.angle_increment
            if ang < -self.sector or ang > self.sector:
                continue
            total += 1
            if not math.isfinite(r) or r <= 0.0 or r < scan.range_min or r > scan.range_max:
                continue
            pts.append((r * math.cos(ang), r * math.sin(ang)))
        self.sector_rays = total
        if total:
            self.coverage.append(len(pts) / total)

        fit = fit_line(pts, self.tol, self.iters)
        if fit is None:
            self.n_nofit += 1
            return
        nx, ny, c, inliers, rms = fit
        self.dists.append(c)
        self.bearings.append(math.degrees(math.atan2(ny, nx)))
        self.rms.append(rms)
        self.inlier_frac.append(len(inliers) / len(pts))


def spread(vals):
    if not vals:
        return None, None
    if len(vals) == 1:
        return vals[0], 0.0
    return statistics.fmean(vals), statistics.stdev(vals)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/scan')
    ap.add_argument('--scans', type=int, default=40, help='scans to fit (default 40)')
    ap.add_argument('--sector-deg', type=float, default=20.0,
                    help='half-width of the forward sector, deg (default 20, '
                         'matching the reflex collision-stop)')
    ap.add_argument('--inlier-tol', type=float, default=0.010,
                    help='RANSAC inlier tolerance, m (default 0.010)')
    ap.add_argument('--iters', type=int, default=200, help='RANSAC iterations (default 200)')
    ap.add_argument('--timeout', type=float, default=30.0, help='seconds to wait (default 30)')
    ap.add_argument('--overhang', action='store_true',
                    help='also print bumper clearance using FRONT_OVERHANG '
                         '(SUSPECT since the 2026-08-16 camera rotation)')
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = WallProbe(args.topic, math.radians(args.sector_deg), args.inlier_tol, args.iters)
    deadline = time.time() + args.timeout
    while rclpy.ok() and node.n_scans < args.scans and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    d_mean, d_sd = spread(node.dists)
    b_mean, b_sd = spread(node.bearings)
    r_mean, _ = spread(node.rms)
    i_mean, _ = spread(node.inlier_frac)
    c_mean, _ = spread(node.coverage)

    if args.json:
        print(json.dumps({
            'topic': args.topic, 'frame_id': node.frame_id,
            'scans_seen': node.n_scans, 'scans_fitted': len(node.dists),
            'scans_no_fit': node.n_nofit, 'sector_deg': args.sector_deg,
            'sector_rays': node.sector_rays,
            'distance_m': d_mean, 'distance_sd_m': d_sd,
            'bearing_deg': b_mean, 'bearing_sd_deg': b_sd,
            'fit_rms_m': r_mean, 'inlier_fraction': i_mean, 'coverage': c_mean,
        }, indent=2))
    else:
        print()
        print('WALL PROBE -- absolute reference, wheel-independent')
        print(f'  topic {args.topic}   frame {node.frame_id or "(none seen)"}')
        if node.angle_min is not None:
            print(f'  scan spans {math.degrees(node.angle_min):+.1f} .. '
                  f'{math.degrees(node.angle_max):+.1f} deg over {node.n_rays_total} rays')
        print(f'  scans seen {node.n_scans}/{args.scans}   fitted {len(node.dists)}   '
              f'no fit {node.n_nofit}')
        print(f'  sector +/-{args.sector_deg:.1f} deg -> {node.sector_rays} rays')
        print()
        if not node.n_scans:
            print('  !! NO SCANS RECEIVED. /scan is not publishing -- do not read this as')
            print('     "nothing in front of the rover". Prove a non-zero baseline first.')
        elif not node.dists:
            print('  !! SCANS ARRIVED BUT NO LINE COULD BE FITTED.')
            print('     Either nothing valid is in the sector, or what is there is not a wall.')
            if c_mean is not None:
                print(f'     sector coverage {c_mean:.2f}')
        else:
            print(f'  perpendicular distance : {d_mean:.4f} m   sd {d_sd*1000:.1f} mm')
            print(f'  bearing to wall normal : {b_mean:+.2f} deg  sd {b_sd:.2f}   '
                  f'(0 = square to the wall)')
            print(f'  fit RMS residual       : {r_mean*1000:.1f} mm')
            print(f'  inlier fraction        : {i_mean:.2f}')
            print(f'  sector coverage        : {c_mean:.2f}')
            if args.overhang:
                print()
                print(f'  front_overhang         : {FRONT_OVERHANG:.3f} m  '
                      f'(SUSPECT since 2026-08-16 -- the camera was rotated)')
                print(f'  implied bumper clearance: {d_mean - FRONT_OVERHANG:.4f} m')
            print()
            # Judge the run, do not just print it.
            if c_mean is not None and c_mean < 0.35:
                print('  !! COVERAGE BELOW THE REFLEX HEALTH GATE (0.35). The collision-stop')
                print('     would call this BLIND. Distance above is fitted to very few rays.')
            if r_mean > 0.010:
                print('  !! FIT RMS > 10 mm -- this is probably not a flat wall. Check for a')
                print('     corner, doorway, skirting or furniture inside the sector.')
            if d_sd > 0.005:
                print('  !! DISTANCE sd > 5 mm -- the manuals quote ~1 mm for a good run.')
                print('     Something moved, or the camera is being starved. Re-run quiet.')
            if abs(b_mean) > 2.0:
                print(f'  -> NOT SQUARE: yawed {abs(b_mean):.1f} deg off the wall normal.')
                print('     Distance is still valid (it is perpendicular), but re-square the')
                print('     rover before deriving any front-facing geometry from it.')
        print()

    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.dists else 1


if __name__ == '__main__':
    sys.exit(main())
