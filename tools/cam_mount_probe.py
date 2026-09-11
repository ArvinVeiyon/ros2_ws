#!/usr/bin/env python3
"""Measure the camera's mounted PITCH and ROLL from gravity. Passive.

Subscribes to /camera/accel/sample and nothing else. Commands nothing, never arms,
does not touch the FC. Safe to run at any time.

WHY THIS TOOL EXISTS
    cam_pitch / cam_roll in launch/depth_to_scan.launch.py feed the base_link ->
    camera_link static TF, which decides where /scan points land -- and the
    collision reflex acts on /scan. The launch comment documents the failure
    mode: too much downward pitch and bare floor enters /scan at 3.4 m instead
    of 6.25 m, reading as a wall dead ahead.

    Two measurements of the same quantity disagree:
      2026-07-27 (launch file comment): pitch 0.0406 rad, roll 0.0100 rad,
                 4561 samples, |g| 9.774. Voids the 07-21 values as pre-remount.
      2026-09-04 (memory):              "level ~1 deg", comparing |g| 9.7864
                 against 9.787 measured on 07-21 -- i.e. against the baseline
                 the launch file calls VOID.
    This reproduces the 07-27 method exactly so a third reading is comparable.

METHOD
    At rest the accelerometer reads the reaction to gravity, so the normalised
    acceleration IS the up-vector. The topic is camera_accel_OPTICAL_frame
    (x right, y down, z forward); camera_link is (x forward, y left, z up):
        x_link = z_opt      y_link = -x_opt      z_link = -y_opt
    Then, for the up-vector u:
        pitch = atan2(-u_x, u_z)   positive = NOSE DOWN
        roll  = atan2( u_y, u_z)   positive = LEFT SIDE UP
    Yaw is unobservable from gravity.

READ THIS BEFORE TRUSTING A NUMBER
    - The rover must be STILL and on FLAT, LEVEL floor. Gravity cannot separate
      a tilted camera from a tilted rover; a sloped floor biases pitch directly.
    - Judge the run by the printed sd and by |g| sitting near 9.81, NOT by
      agreement with any previous figure.
    - On stands or lifted, the result does not describe the driving geometry.

Usage:
    python3 tools/cam_mount_probe.py                # 60 s
    python3 tools/cam_mount_probe.py --seconds 120  # tighter statistics
"""
import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu

# Current values in launch/depth_to_scan.launch.py (MEASURED 2026-07-27).
LAUNCH_PITCH = 0.0406
LAUNCH_ROLL = 0.0100


class Probe(Node):
    def __init__(self):
        super().__init__('cam_mount_probe')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.x, self.y, self.z = [], [], []
        self.create_subscription(Imu, '/camera/accel/sample', self.cb, qos)

    def cb(self, m):
        a = m.linear_acceleration
        self.x.append(a.x)
        self.y.append(a.y)
        self.z.append(a.z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    a = ap.parse_args()

    rclpy.init()
    n = Probe()
    print(f'sampling /camera/accel/sample for {a.seconds:.0f} s -- KEEP THE ROVER STILL', flush=True)
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    N = len(n.x)
    if N < 100:
        print(f'\nONLY {N} SAMPLES. Not enough -- the topic is quiet or the run was too short.')
        print('A quiet topic is NOT evidence of anything. Fix the camera first.')
        rclpy.shutdown()
        sys.exit(1)

    mx, my, mz = statistics.fmean(n.x), statistics.fmean(n.y), statistics.fmean(n.z)
    sx, sy, sz = (statistics.pstdev(v) for v in (n.x, n.y, n.z))
    g = math.sqrt(mx * mx + my * my + mz * mz)

    # optical -> link
    ux, uy, uz = mz, -mx, -my
    ux, uy, uz = ux / g, uy / g, uz / g
    pitch = math.atan2(-ux, uz)
    roll = math.atan2(uy, uz)

    print(f'\n=== {N} samples ===')
    print(f'optical mean  x {mx:+.4f}  y {my:+.4f}  z {mz:+.4f}   (sd {sx:.4f} {sy:.4f} {sz:.4f})')
    print(f'|g| {g:.4f}  vs 9.81 nominal   ' + ('OK' if 9.6 < g < 10.0 else 'SUSPECT -- do not trust this run'))
    print(f'up-vector in camera_link ({ux:+.4f}, {uy:+.4f}, {uz:+.4f})')
    print()
    print(f'  MEASURED pitch {pitch:+.4f} rad = {math.degrees(pitch):+.3f} deg  (positive = NOSE DOWN)')
    print(f'  MEASURED roll  {roll:+.4f} rad = {math.degrees(roll):+.3f} deg  (positive = LEFT SIDE UP)')
    print()
    print(f'  launch cam_pitch {LAUNCH_PITCH:+.4f} rad = {math.degrees(LAUNCH_PITCH):+.3f} deg'
          f'   delta {math.degrees(pitch - LAUNCH_PITCH):+.3f} deg')
    print(f'  launch cam_roll  {LAUNCH_ROLL:+.4f} rad = {math.degrees(LAUNCH_ROLL):+.3f} deg'
          f'   delta {math.degrees(roll - LAUNCH_ROLL):+.3f} deg')
    print()
    print('  for reference: 2026-07-27 measured pitch 2.326 deg / roll 0.573 deg (4561 samples)')
    print('                 2026-09-04 memory note claimed "level ~1 deg" for pitch')
    dp = abs(math.degrees(pitch - LAUNCH_PITCH))
    if dp < 0.30:
        print(f'\n=> AGREES with the launch file ({dp:.3f} deg apart). The 09-04 "0.87 deg" note')
        print('   does not reproduce. Leave the launch values alone.')
    else:
        print(f'\n=> DISAGREES with the launch file by {dp:.3f} deg.')
        print('   At 4 m that is about %.3f m of vertical error in where /scan points land.' % (4.0 * abs(math.tan(pitch - LAUNCH_PITCH))))
        print('   Changing it is SAFETY-RELEVANT: re-verify with wall_probe.py afterwards.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
