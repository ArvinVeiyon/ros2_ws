import rclpy
from rclpy.node import Node
from px4_msgs.msg import InputRc
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import subprocess

class CameraNodeSw(Node):
    def __init__(self):
        super().__init__('camera_node_sw')

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
        # Track the last camera state to avoid redundant switches
        self.last_camera_state = None

    def switch_camera(self, primary_device, secondary_device=None):
        """Call vision_config_manager to switch camera(s)."""
        cmd = ['sudo', '/usr/local/bin/vision_config_manager', primary_device]
        if secondary_device:
            cmd.append(secondary_device)
        
        try:
            subprocess.run(cmd, check=True)
            if secondary_device:
                self.get_logger().info(f"Switched to split view: {primary_device} (front) and {secondary_device} (bottom)")
            else:
                self.get_logger().info(f"Switched to camera: {primary_device}")
            self.last_camera_state = (primary_device, secondary_device)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to switch cameras: {e}")

    def listener_callback(self, msg):
        self.get_logger().info("Callback triggered - RC input received.")

        # Print all channels, counting from 1 instead of 0
        for index, value in enumerate(msg.values, start=1):
            self.get_logger().info(f"Channel {index}: {value}")

        # Check channel 7 value
        channel_7_value = msg.values[6]  # Channel 7 is index 6 in 0-indexed arrays
        self.get_logger().info(f"Channel 7 value is: {channel_7_value}")

        # Define camera states
        front_camera = '/dev/video0'
        bottom_camera = '/dev/video2'

        # Determine the desired camera state based on channel 7 value
        if channel_7_value == 1024:
            desired_state = (front_camera, None)  # Front camera only
        elif channel_7_value == 1514:
            desired_state = (bottom_camera, None)  # Bottom camera only
        elif channel_7_value == 2003:
            desired_state = (front_camera, bottom_camera)  # Split view (front + bottom)
        else:
            return  # Ignore other values

        # Only switch if the state has changed
        if self.last_camera_state != desired_state:
            if desired_state[1] is None:
                self.switch_camera(desired_state[0])  # Single camera
            else:
                self.switch_camera(desired_state[0], desired_state[1])  # Split view

def main(args=None):
    rclpy.init(args=args)
    node = CameraNodeSw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
