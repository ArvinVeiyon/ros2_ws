#!/usr/bin/env python3
import os
import argparse
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import InputRc

class RCChangeCapture(Node):
    def __init__(self, channel_num, output_file=None):
        super().__init__('rc_change_capture')
        # convert 1‑based channel to 0‑based array index
        self.channel = channel_num - 1  
        self.last = None
        self.output_file = output_file

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.cb,
            qos_profile=qos
        )
        self.get_logger().info(
            f"Watching RC channel {channel_num} (values[{self.channel}]) for changes"
        )

    def cb(self, msg: InputRc):
        val = int(msg.values[self.channel])
        if self.last is None:
            self.last = val
            return
        if val != self.last:
            self.get_logger().info(f"Channel {self.channel+1}: {self.last} → {val}")
            if self.output_file:
                # write only the changed channel/value
                with open(self.output_file, 'w') as f:
                    yaml.dump({str(self.channel+1): val}, f)
                self.get_logger().info(f"Wrote change to {self.output_file}")
            self.last = val

def main():
    parser = argparse.ArgumentParser(
        description="Test script: capture only channel changes (1‑based)"
    )
    parser.add_argument(
        '-c','--channel', type=int, default=1,
        choices=range(1,19),
        help="RC channel number to watch (1–18, default 1)"
    )
    parser.add_argument(
        '-o','--output', default=None,
        help="Optional YAML file to write each change"
    )
    args = parser.parse_args()

    rclpy.init()
    node = RCChangeCapture(args.channel, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
