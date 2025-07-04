import rclpy
from rclpy.node import Node
from px4_msgs.msg import InputRc
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import subprocess

class ShutdownRebootNode(Node):
    def __init__(self):
        super().__init__('shutdown_reboot_node')

        # QoS settings for the subscriber
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        print("Callback triggered - RC input received.")

        # Print all channels, counting from 1 instead of 0
        for index, value in enumerate(msg.values, start=1):
            print(f"Channel {index}: {value}")

        # Check channel 7 value
        channel_7_value = msg.values[6]  # Channel 7 is index 6 in 0-indexed arrays
        print(f"Channel 7 value is: {channel_7_value}")

        # Directly execute shutdown or reboot based on channel 7 value
        if channel_7_value == 2003:
            print("System will shut down now...")
            try:
                subprocess.run(['sudo', '/sbin/shutdown', 'now'], check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error during shutdown: {e}")
        
        elif channel_7_value == 1514:
            print("System will reboot now...")
            try:
                subprocess.run(['sudo', '/sbin/reboot'], check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error during reboot: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ShutdownRebootNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
