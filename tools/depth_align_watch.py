#!/usr/bin/env python3
"""Catch the intermittent unaligned depth frame that kills RTAB-Map localization.

With `depth_registration:=true` and `align_mode:=SW` (the gemini_330_series launch
default) the wrapper aligns depth to colour in software and publishes the result on
/camera/depth/image_raw. When a frameset arrives WITHOUT a usable colour frame the
alignment is skipped -- and the raw, unaligned depth frame is published on that same
topic anyway. It is 1280x800 instead of 640x360, and RTAB-Map's
Memory.cpp::createSignature() asserts depth <= colour, so the process ABORTS.

This watches for such a frame and records what colour was doing around it, which
discriminates "the frameset had no colour frame" from "colour was flowing normally".

Colour is watched via camera_info, not image_raw: same one-per-frame cadence, a few
hundred bytes instead of ~10 MB/s on a Pi that is already core-starved.

    python3 tools/depth_align_watch.py --minutes 30
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class DepthAlignWatch(Node):
    def __init__(self, report_s):
        super().__init__('depth_align_watch')
        self.depth_n = 0
        self.color_n = 0
        self.sizes = {}
        self.expected = None
        self.events = []
        self.last_color_t = None
        self.prev_depth_t = None
        self.started = time.monotonic()
        self.report_s = report_s
        self.last_report = self.started

        self.create_subscription(
            Image, '/camera/depth/image_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.on_color, qos_profile_sensor_data)
        self.create_timer(1.0, self.tick)

    def on_color(self, msg):
        self.color_n += 1
        self.last_color_t = time.monotonic()

    def on_depth(self, msg):
        now = time.monotonic()
        self.depth_n += 1
        size = (msg.width, msg.height)
        self.sizes[size] = self.sizes.get(size, 0) + 1

        # The first size seen is the aligned one; the glitch is rare by construction.
        if self.expected is None:
            self.expected = size
            self.get_logger().info(f'aligned depth size = {size[0]}x{size[1]}')
        elif size != self.expected:
            # How stale was colour when this frame was published? If the frameset
            # genuinely had no colour frame, this gap is one frame period or more.
            gap = None if self.last_color_t is None else now - self.last_color_t
            since_prev = None if self.prev_depth_t is None else now - self.prev_depth_t
            self.events.append((now - self.started, size, gap))
            self.get_logger().error(
                f'UNALIGNED DEPTH at t={now - self.started:.1f}s: '
                f'{size[0]}x{size[1]} (expected {self.expected[0]}x{self.expected[1]}) | '
                f'colour last seen {"never" if gap is None else f"{gap * 1000:.0f} ms"} ago | '
                f'previous depth {"n/a" if since_prev is None else f"{since_prev * 1000:.0f} ms"} ago')
        self.prev_depth_t = now

    def tick(self):
        now = time.monotonic()
        if now - self.last_report < self.report_s:
            return
        self.last_report = now
        el = now - self.started
        sizes = ' '.join(f'{w}x{h}:{n}' for (w, h), n in sorted(self.sizes.items()))
        self.get_logger().info(
            f'[{el / 60:.1f} min] depth {self.depth_n} ({self.depth_n / el:.1f} Hz) '
            f'colour {self.color_n} ({self.color_n / el:.1f} Hz) | sizes {sizes} | '
            f'unaligned events {len(self.events)}')

    def summary(self):
        el = time.monotonic() - self.started
        print(f'\n=== {el / 60:.1f} min ===')
        print(f'depth   {self.depth_n} msgs, {self.depth_n / el:.2f} Hz')
        print(f'colour  {self.color_n} msgs, {self.color_n / el:.2f} Hz')
        for size, n in sorted(self.sizes.items()):
            print(f'  depth {size[0]}x{size[1]}: {n}')
        print(f'unaligned depth frames: {len(self.events)}')
        for t, size, gap in self.events:
            g = 'never' if gap is None else f'{gap * 1000:.0f} ms'
            print(f'  t={t:.1f}s  {size[0]}x{size[1]}  colour age {g}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutes', type=float, default=30.0)
    ap.add_argument('--report', type=float, default=120.0, help='progress line interval (s)')
    args = ap.parse_args()

    rclpy.init()
    node = DepthAlignWatch(args.report)
    deadline = time.monotonic() + args.minutes * 60
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        # Ctrl-C shuts the context down under us, and shutting it down twice raises.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
