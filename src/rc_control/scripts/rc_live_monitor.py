#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import InputRc

class RCLiveMonitor(Node):
    def __init__(self):
        super().__init__('rc_live_monitor')
        self.last = None

        # PX4 publishes with Best‑Effort reliability, so match it here
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

    def cb(self, msg: InputRc):
        vals = [int(v) for v in msg.values]
        if self.last is not None:
            for i, (old, new) in enumerate(zip(self.last, vals)):
                if new != old:
                    self.get_logger().info(f"Channel {i}: {old} → {new}")
        self.last = vals

def main():
    rclpy.init()
    node = RCLiveMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
