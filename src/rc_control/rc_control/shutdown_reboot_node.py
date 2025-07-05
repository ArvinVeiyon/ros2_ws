#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import InputRc
import subprocess

class ShutdownRebootNode(Node):
    def __init__(self):
        super().__init__('shutdown_reboot_node')

        # QoS: best effort, keep last
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to PX4's input_rc topic
        topic_name = '/fmu/out/input_rc'
        self.create_subscription(
            InputRc,
            topic_name,
            self.cb_rc,
            qos
        )
        self.get_logger().info(f'Subscribed to {topic_name}')

        # State for hold detection
        self.shutdown_start = None
        self.reboot_start = None

        # Flags to ensure one-shot
        self.did_shutdown = False
        self.did_reboot = False

        # Midpoint and tolerance
        self.MID = 1024
        self.TOL = 15  # tolerance
        # Required hold time (seconds)
        self.HOLD_TIME = 1.0

        self.get_logger().info('Shutdown/Reboot node ready (hold sticks to trigger)')

    def cb_rc(self, msg: InputRc):
        vals = msg.values
        # Ensure 10 channels
        if len(vals) < 10:
            return

        ch9, ch10 = vals[8], vals[9]
        mid9 = abs(ch9 - self.MID) <= self.TOL
        mid10 = abs(ch10 - self.MID) <= self.TOL

        now = self.get_clock().now().nanoseconds * 1e-9

        # Shutdown: both mid held
        if mid9 and mid10 and not self.did_shutdown:
            if self.shutdown_start is None:
                self.shutdown_start = now
                self.get_logger().info('Shutdown sequence armed; hold both sticks at mid')
            elif (now - self.shutdown_start) >= self.HOLD_TIME:
                self.get_logger().warn('Held both sticks → SHUTDOWN')
                subprocess.Popen(['sudo', '/sbin/shutdown', '-h', 'now'])
                self.did_shutdown = True
        else:
            # reset if sticks moved
            self.shutdown_start = None

        # Reboot: only channel 10 mid held
        if mid10 and not mid9 and not self.did_reboot:
            if self.reboot_start is None:
                self.reboot_start = now
                self.get_logger().info('Reboot sequence armed; hold channel 10 at mid')
            elif (now - self.reboot_start) >= self.HOLD_TIME:
                self.get_logger().warn('Held stick 10 → REBOOT')
                subprocess.Popen(['sudo', '/sbin/reboot'])
                self.did_reboot = True
        else:
            self.reboot_start = None


def main(args=None):
    rclpy.init(args=args)
    node = ShutdownRebootNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
