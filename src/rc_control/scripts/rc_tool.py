#!/usr/bin/env python3
import os
import argparse
import threading
import time
from ruamel.yaml import YAML

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import InputRc

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURE HERE: how many channels to skip (1‑based)
# Skip channels 1–4 (values[0]–[3]); start at channel 5 (values[4])
SKIP_UP_TO = 4
# ──────────────────────────────────────────────────────────────────────────────

# locate your YAML
SCRIPT_DIR   = os.path.dirname(os.path.realpath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
YAML_PATH    = os.path.join(PROJECT_ROOT, 'config', 'rc_mapping.yaml')

class RCTool(Node):
    def __init__(self, mode):
        super().__init__('rc_tool')
        self.mode   = mode
        self.latest = None

        # load (or init) YAML
        self.yaml = YAML()
        with open(YAML_PATH) as f:
            self.data = self.yaml.load(f) or {}

        # subscribe to PX4’s RC topic with Best‑Effort QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            InputRc, '/fmu/out/input_rc',
            lambda msg: setattr(self, 'latest', msg),
            qos_profile=qos
        )

        if mode == 'monitor':
            self.data.setdefault('rc_values', {'timestamp':0,'values':[]})
            self.last = None
            self.get_logger().info("Monitor mode: dumping all channels")

        elif mode == 'positions':
            # build list of 1‑based channels > SKIP_UP_TO
            raw = sorted(map(int, self.data.get('rc_channels', {})))
            self.ch_list = [i+1 for i in raw if (i+1) > SKIP_UP_TO]
            if not self.ch_list:
                self.get_logger().error(f"No channels >{SKIP_UP_TO} defined in rc_channels")
                raise SystemExit(1)
            self.get_logger().info(f"Positions mode: channels {self.ch_list}")

        else:  # node‑calibrate
            self.plan = [
                nm for nm, blk in self.data.get('nodes', {}).items()
                if blk.get('ros__parameters', {}).get('rc_map') is not None
            ]
            if not self.plan:
                self.get_logger().error("No nodes with rc_map in YAML")
                raise SystemExit(1)
            self.get_logger().info(f"Node‑calibrate mode: {self.plan}")

    def start_bg_spin(self):
        """Keep rclpy.spin running during input() calls."""
        th = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        th.start()

    def run_monitor(self):
        self.get_logger().info("Entering monitor mode (CTRL+C to exit)")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.latest:
                continue
            vals = [int(v) for v in self.latest.values]
            if vals != self.last:
                self.data['rc_values']['timestamp'] = int(self.latest.timestamp)
                self.data['rc_values']['values']    = vals
                self.last = vals
                with open(YAML_PATH,'w') as f:
                    self.yaml.dump(self.data, f)
                self.get_logger().info(f"rc_values updated: {vals}")

    def run_positions(self):
        self.start_bg_spin()

        # 1) Snapshot pos1 for all channels > SKIP_UP_TO
        self.get_logger().info(f"Snapshot pos1 for channels {self.ch_list}…")
        while self.latest is None and rclpy.ok():
            time.sleep(0.1)
        snap = [int(v) for v in self.latest.values]
        for ch in self.ch_list:
            idx = ch - 1
            self.data['rc_channels'][str(idx)]['pos1'] = snap[idx]
        with open(YAML_PATH,'w') as f:
            self.yaml.dump(self.data, f)
        self.get_logger().info(f"pos1 set for channels: {self.ch_list}")

        # 2) Prompt each channel for pos2 & pos3
        for ch in self.ch_list:
            idx = ch - 1
            slot = self.data['rc_channels'][str(idx)]
            self.get_logger().info(f"--- Calibrating Channel {ch} ---")

            for pos in ('pos2','pos3'):
                if pos not in slot:
                    continue

                curr = int(self.latest.values[idx])
                print(f"\nChannel {ch} current PWM = {curr}")
                ans = input(f"Move stick to '{pos}', hold, then Enter (s=skip) → ")
                if ans.strip().lower().startswith('s'):
                    slot[pos] = None
                    self.get_logger().info(f"Skipped CH{ch}.{pos}")
                else:
                    time.sleep(0.2)
                    slot[pos] = int(self.latest.values[idx])
                    self.get_logger().info(f"Captured CH{ch}.{pos} = {slot[pos]}")

                with open(YAML_PATH,'w') as f:
                    self.yaml.dump(self.data, f)

        self.get_logger().info("Position calibration complete.")

    def run_nodecal(self):
        self.start_bg_spin()

        for nm in self.plan:
            params = self.data['nodes'][nm]['ros__parameters']
            tol = params.get('tolerance', 100)

            # always re‑prompt for channel mapping
            ans = input(f"[{nm}] Auto‑detect channel? (y/n) → ")
            if ans.lower().startswith('y'):
                input("Wiggle stick now, then press Enter → ")
                while self.latest is None:
                    time.sleep(0.1)
                base = list(self.latest.values)

                det = None
                for _ in range(200):
                    cur = list(self.latest.values)
                    deltas = [abs(cur[i] - base[i]) for i in range(len(cur))]
                    cands = [i for i, d in enumerate(deltas) if d > tol]
                    if len(cands) == 1:
                        det = cands[0] + 1
                        break
                    time.sleep(0.05)

                if det is None:
                    det = int(input(f"No clear movement >{tol}, enter channel (1–18) → "))
                ch = det
            else:
                ch = int(input(f"[{nm}] Enter channel (1–18) → "))

            params['channel_index'] = ch
            with open(YAML_PATH,'w') as f:
                self.yaml.dump(self.data, f)
            self.get_logger().info(f"{nm}.channel_index set to CH{ch}")

            idx = ch - 1
            for act in params['rc_map']:
                input(f"[{nm}] Move stick for '{act}' on CH{ch}, then Enter → ")
                time.sleep(0.2)
                pwm = int(self.latest.values[idx])
                params['rc_map'][act] = pwm
                self.get_logger().info(f"{nm}.rc_map['{act}'] = {pwm}")
                with open(YAML_PATH,'w') as f:
                    self.yaml.dump(self.data, f)

        self.get_logger().info("Node calibration complete.")

def main():
    parser = argparse.ArgumentParser(
        description="RC Tool: -m monitor, -p positions, -c node‑calibrate"
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-m','--monitor',   action='store_true')
    group.add_argument('-p','--positions', action='store_true')
    group.add_argument('-c','--nodecal',   action='store_true')
    args = parser.parse_args()

    rclpy.init()
    mode = 'monitor' if args.monitor else 'positions' if args.positions else 'nodecal'
    tool = RCTool(mode)
    try:
        if args.monitor:
            tool.run_monitor()
        elif args.positions:
            tool.run_positions()
        else:
            tool.run_nodecal()
    except KeyboardInterrupt:
        pass
    tool.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
