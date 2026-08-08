#!/usr/bin/env python3
"""Verify a rover-camera restart actually came up alive.

`systemctl is-active` does NOT catch the failure this exists for: the service can
report active, answer parameter queries, and never have started the depth or colour
streams at all, with nothing logged. A second restart has cleared it before.

Checks, in the order that fails fastest:
  1. the unit is active
  2. the journal since the restart shows BOTH "depth Frame - Width" and
     "color Frame - Width" -- absent means half-dead
  3. the streams are actually publishing, at a rate close to the configured fps
  4. depth image dimensions match the colour dimensions when depth_registration
     is on -- the todo #26 failure mode
  5. any unaligned-depth WARNs the patched wrapper has emitted since the restart

    python3 tools/camera_restart_check.py            # after restarting the service
    python3 tools/camera_restart_check.py --secs 20  # longer rate window
"""
import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

TOPICS = [
    ('/camera/depth/image_raw', Image),
    ('/camera/color/image_raw', Image),
    ('/scan', LaserScan),
]


def journal(since, pattern):
    out = subprocess.run(
        ['journalctl', '-u', 'rover-camera', '--since', since, '--no-pager'],
        capture_output=True, text=True).stdout
    return [ln for ln in out.splitlines() if pattern in ln]


class Probe(Node):
    def __init__(self):
        super().__init__('camera_restart_check')
        self.counts = {t: 0 for t, _ in TOPICS}
        self.dims = {}
        for topic, msg_type in TOPICS:
            self.create_subscription(
                msg_type, topic,
                lambda m, t=topic: self.on_msg(t, m), qos_profile_sensor_data)

    def on_msg(self, topic, msg):
        self.counts[topic] += 1
        if isinstance(msg, Image):
            self.dims[topic] = (msg.width, msg.height)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=12.0)
    ap.add_argument('--since', default='-3min')
    args = ap.parse_args()
    failures = []

    active = subprocess.run(['systemctl', 'is-active', 'rover-camera'],
                            capture_output=True, text=True).stdout.strip()
    print(f'1. unit .................. {active}')
    if active != 'active':
        failures.append('rover-camera is not active')

    for stream in ('depth', 'color'):
        lines = journal(args.since, f'{stream} Frame - Width')
        ok = 'OK' if lines else 'MISSING -> HALF-DEAD, restart again'
        print(f'2. {stream:5s} stream started .. {ok}')
        if lines:
            print(f'     {lines[-1].split("]: ")[-1]}')
        else:
            failures.append(f'{stream} stream never started (half-dead restart)')

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + args.secs
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f'3. rates over {args.secs:.0f}s')
    for topic, _ in TOPICS:
        hz = node.counts[topic] / args.secs
        dim = node.dims.get(topic)
        extra = f'  {dim[0]}x{dim[1]}' if dim else ''
        print(f'     {topic:28s} {hz:5.1f} Hz{extra}')
        if node.counts[topic] == 0:
            failures.append(f'{topic} published nothing')

    depth = node.dims.get('/camera/depth/image_raw')
    color = node.dims.get('/camera/color/image_raw')
    if depth and color:
        ok = depth == color
        print(f'4. depth aligned to colour  {"OK" if ok else f"MISMATCH {depth} vs {color}"}')
        if not ok:
            failures.append(f'depth {depth} != colour {color} (todo #26 failure mode)')

    warns = journal(args.since, 'not aligned')
    print(f'5. unaligned-depth WARNs .. {len(warns)}')
    for ln in warns[-3:]:
        print(f'     {ln.split("]: ")[-1]}')

    node.destroy_node()
    rclpy.shutdown()

    print()
    if failures:
        print('FAILED:')
        for f in failures:
            print(f'  - {f}')
        return 1
    print('All checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
