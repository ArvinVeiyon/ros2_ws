#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorOpticalFlow, DistanceSensor, SensorCombined
import cv2
import numpy as np

class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__('optical_flow_node')

        # Publisher for SensorOpticalFlow messages
        self.flow_publisher = self.create_publisher(SensorOpticalFlow, '/fmu/in/sensor_optical_flow', 10)

        # Subscribe to the TFmini distance sensor topic (if applicable)
        self.distance_subscriber = self.create_subscription(
            DistanceSensor,
            '/fmu/in/distance_sensor',
            self.distance_callback,
            10
        )

        # Subscribe to the gyro data from the FMU (SensorCombined message)
        self.gyro_subscriber = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.gyro_callback,
            10
        )

        # Initialize distance and gyro data
        self.current_distance = None
        self.gyro_data = [0.0, 0.0, 0.0]

        # Initialize camera to capture frames
        self.cap = cv2.VideoCapture('/dev/video3')  # Adjust the device path for your camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error('Error: Camera not found or could not be opened.')
            return

        # Capture the first frame
        ret, self.prev_frame = self.cap.read()
        if not ret:
            self.get_logger().error('Error: Failed to capture the first frame from the camera.')
            return

        # Convert the first frame to grayscale
        self.prev_gray = cv2.cvtColor(self.prev_frame, cv2.COLOR_BGR2GRAY)

        # Timer to process and publish optical flow at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_optical_flow)

    def distance_callback(self, msg):
        # Store the current distance in meters from the TFmini sensor (if applicable)
        self.current_distance = msg.current_distance

    def gyro_callback(self, msg):
        if msg.gyro_integral_dt > 0:
            # Convert gyro_rad (radians/sec) and gyro_integral_dt (microseconds) to delta angle (radians)
            self.gyro_data = [
                msg.gyro_rad[0] * (msg.gyro_integral_dt / 1e6),  # X-axis delta angle in radians
                msg.gyro_rad[1] * (msg.gyro_integral_dt / 1e6),  # Y-axis delta angle in radians
                msg.gyro_rad[2] * (msg.gyro_integral_dt / 1e6)   # Z-axis delta angle in radians
            ]
            # Log the received gyro data and calculated delta angles
            self.get_logger().info(f"Gyro data: {msg.gyro_rad}, Delta Angle: {self.gyro_data}")
        else:
            self.get_logger().warn('gyro_integral_dt is zero or negative!')

    def publish_optical_flow(self):
        # Capture the current frame
        ret, curr_frame = self.cap.read()
        if not ret:
            self.get_logger().error('Error: Failed to capture image from the camera.')
            return

        # Convert current frame to grayscale
        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

        # Calculate optical flow between previous and current frames
        flow = cv2.calcOpticalFlowFarneback(self.prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

        # Extract flow in X and Y directions
        flow_x = flow[..., 0]
        flow_y = flow[..., 1]

        # Compute average flow values
        avg_flow_x = np.mean(flow_x)
        avg_flow_y = np.mean(flow_y)

        # Prepare the SensorOpticalFlow message
        msg = SensorOpticalFlow()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Timestamp in microseconds
        msg.timestamp_sample = msg.timestamp
        msg.pixel_flow = [avg_flow_x, avg_flow_y]
        msg.integration_timespan_us = 100000  # 100 ms integration time
        msg.quality = 255  # Maximum quality
        msg.min_ground_distance = 0.1
        msg.max_ground_distance = 5.0

        # Use the TFmini distance data if available
        if self.current_distance is not None:
            msg.distance_m = self.current_distance
            msg.distance_available = True
        else:
            msg.distance_m = float('nan')  # Use NaN for unavailable distance
            msg.distance_available = False

        # Use the gyro data from the FMU for delta_angle (calculated using gyro_integral_dt)
        msg.delta_angle = self.gyro_data
        msg.delta_angle_available = True

        # Set the device ID (replace with your actual sensor ID)
        msg.device_id = 123456  # Example device ID, update with real sensor ID

        # Publish the optical flow message to PX4
        self.flow_publisher.publish(msg)
        self.get_logger().info(f'Published optical flow with distance: {msg.distance_m} meters and gyro: {msg.delta_angle}')

        # Set current frame as previous frame for next calculation
        self.prev_gray = curr_gray

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
