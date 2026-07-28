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
from px4_msgs.msg import EscStatus
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
        self.rpm = [0, 0, 0, 0]
        self.burst = None
        self.bursts = []

    def esc_cb(self, msg):
        self.rpm = [msg.esc[i].esc_rpm for i in range(4)]

    def odom_cb(self, msg):
        now = time.time()
        wz = msg.twist.twist.angular.z
        vx = msg.twist.twist.linear.x
        moving = abs(wz) > MOVING_YAW or abs(vx) > MOVING_FWD

        if moving:
            if self.burst is None:
                self.burst = {'t0': now, 'wz': [], 'vx': [], 'rpm': [0, 0, 0, 0]}
            b = self.burst
            b['last'] = now
            b['wz'].append(wz)
            b['vx'].append(vx)
            b['rpm'] = [max(a, abs(r)) for a, r in zip(b['rpm'], self.rpm)]
        elif self.burst and now - self.burst.get('last', now) > GAP_S:
            self.close_burst()

    def close_burst(self):
        b = self.burst
        self.burst = None
        if not b or len(b['wz']) < 5:
            return
        peak_wz = max(b['wz'], key=abs)
        peak_vx = max(b['vx'], key=abs)
        # mean over the sustained middle half, skipping accel/decel ramps
        mid = sorted(b['wz'], key=abs)[len(b['wz']) // 2:]
        mean_wz = sum(mid) / len(mid)
        self.bursts.append(b)
        print(f'burst {len(self.bursts)}: {b["last"] - b["t0"]:.1f}s  '
              f'yaw peak={peak_wz:+.3f} sustained={mean_wz:+.3f} rad/s  '
              f'fwd peak={peak_vx:+.3f} m/s  rpm={b["rpm"]}', flush=True)


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
