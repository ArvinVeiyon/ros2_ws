#!/usr/bin/env python3
"""Passive recorder for yaw-gain tuning (#20): achieved body yaw rate vs wheel effort.

Run this alongside `l2_test.py --live`. It commands nothing and never arms — it only
listens, so it is safe to leave running for a whole tuning session.

Reports, per motion burst it detects:
  - peak/mean achieved yaw rate from /odom (gyro-sourced, see rover_odometry yaw_source)
  - peak forward speed from /odom
  - peak per-wheel ERPM from esc_status

The number #20 turns on is achieved-vs-commanded yaw rate: if the rover reaches the
commanded rate, the wheel effort is what skid-steer scrub actually costs and the gains
are fine; if it overshoots or hunts, RO_YAW_RATE_P/I are too hot.

Usage: yaw_response_log.py [seconds]     (default 120)
"""
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import EscStatus, SensorCombined
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

MOVING_YAW = 0.02    # [rad/s] above this the rover counts as rotating
MOVING_FWD = 0.02    # [m/s]   above this the rover counts as translating
GAP_S = 0.8          # quiet time that ends a burst


class YawLog(Node):
    def __init__(self):
        super().__init__('yaw_response_log')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc_cb, px4_qos)
        # GROUND TRUTH for yaw rate. /odom's angular.z is NOT trustworthy under motion
        # (2026-07-29: peaked at 62 rad/s, sustained 5.7 rad/s -- physically impossible),
        # so body yaw rate is taken from the raw FC gyro instead. Note the FC does not
        # expose vehicle_angular_velocity over DDS; sensor_combined is the available source.
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined',
                                 self.gyro_cb, px4_qos)
        self.rpm = [0, 0, 0, 0]
        self.gyro_z = 0.0
        self.burst = None
        self.bursts = []

    def esc_cb(self, msg):
        self.rpm = [msg.esc[i].esc_rpm for i in range(4)]

    def gyro_cb(self, msg):
        # PX4 body frame is FRD (+z down), ROS is FLU (+z up) -> negate for a ROS-sense yaw rate.
        self.gyro_z = -msg.gyro_rad[2]

    def odom_cb(self, msg):
        now = time.time()
        wz = msg.twist.twist.angular.z
        vx = msg.twist.twist.linear.x
        moving = abs(wz) > MOVING_YAW or abs(vx) > MOVING_FWD

        if moving:
            if self.burst is None:
                self.burst = {'t0': now, 'wz': [], 'vx': [], 'gz': [], 'rpm': [0, 0, 0, 0]}
            b = self.burst
            b['last'] = now
            b['wz'].append(wz)
            b['vx'].append(vx)
            b['gz'].append(self.gyro_z)
            b['rpm'] = [max(a, abs(r)) for a, r in zip(b['rpm'], self.rpm)]
        elif self.burst and now - self.burst.get('last', now) > GAP_S:
            self.close_burst()

    def close_burst(self):
        b = self.burst
        self.burst = None
        if not b or len(b['wz']) < 5:
            return
        def sustained(xs):
            """Mean of the top half by magnitude — skips the accel/decel ramps."""
            mid = sorted(xs, key=abs)[len(xs) // 2:]
            return sum(mid) / len(mid)

        peak_vx = max(b['vx'], key=abs)
        self.bursts.append(b)
        line = (f'burst {len(self.bursts)}: {b["last"] - b["t0"]:.1f}s  '
                f'fwd peak={peak_vx:+.3f} m/s  rpm={b["rpm"]}')
        if b['gz']:
            line += (f'\n    yaw GYRO  peak={max(b["gz"], key=abs):+.3f} '
                     f'sustained={sustained(b["gz"]):+.3f} rad/s   <-- ground truth')
        line += (f'\n    yaw /odom peak={max(b["wz"], key=abs):+.3f} '
                 f'sustained={sustained(b["wz"]):+.3f} rad/s   (suspect, see gyro_cb)')
        print(line, flush=True)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    n = YawLog()
    print(f'listening {seconds:.0f}s (commands nothing) — run l2_test.py --live now', flush=True)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.close_burst()
    print(f'done: {len(n.bursts)} motion bursts', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
