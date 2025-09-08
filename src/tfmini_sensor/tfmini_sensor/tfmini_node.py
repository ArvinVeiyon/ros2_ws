#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import DistanceSensor
import serial
import math
import time  # added for throttled logging

class TFminiNode(Node):
    def __init__(self):
        super().__init__('tfmini_node')

        # Create publisher for DistanceSensor messages to PX4
        self.publisher_ = self.create_publisher(DistanceSensor, '/fmu/in/distance_sensor', 10)

        # Open serial port for UART communication
        try:
            # Lower timeout for snappier reads (replaces timeout=1)
            self.serial_port = serial.Serial('/dev/ttyAMA2', 115200, timeout=0.001)
            self.get_logger().info('Serial port /dev/ttyAMA2 opened successfully')
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            return

        # Timer callback for publishing distance at 200Hz (0.005 s)
        self.timer = self.create_timer(0.005, self.publish_distance)  # 200 Hz timer

        # Throttled logging counters (avoid per-frame prints)
        self._ok_frames = 0
        self._bad_frames = 0
        self._last_log_sec = time.time()

    def publish_distance(self):
        # Check if enough bytes are available in the serial buffer
        if self.serial_port.in_waiting >= 9:
            try:
                # TFmini sends 9 bytes starting with two 'Y' characters
                if self.serial_port.read() == b'Y' and self.serial_port.read() == b'Y':
                    # Read low and high bytes for the distance measurement
                    low = self.serial_port.read()
                    high = self.serial_port.read()
                    distance = (ord(high) * 256) + ord(low)  # Combine to form the distance in cm
                    self.serial_port.read(5)  # Read remaining unused bytes

                    # Prepare the DistanceSensor message
                    msg = DistanceSensor()
                    msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Timestamp in microseconds
                    msg.device_id = 1987  # Use a unique ID for your TFmini sensor
                    msg.min_distance = 0.3  # Minimum range in meters (kept as-is)
                    msg.max_distance = 12.0  # Maximum range in meters
                    msg.current_distance = distance / 100.0  # Convert distance from cm to meters
                    msg.variance = 0.0  # Set variance (optional, can be tuned)

                    # Signal quality: check if the distance is valid (within sensor range)
                    if 0.3 <= msg.current_distance <= 12.0:
                        msg.signal_quality = 100  # Strong signal
                    else:
                        msg.signal_quality = 0  # Out of range or invalid data

                    # Sensor type: assuming it's a laser-based sensor (TFmini)
                    msg.type = DistanceSensor.MAV_DISTANCE_SENSOR_LASER
                    # NOTE: orientation left unchanged per your request
                    msg.orientation = DistanceSensor.ROTATION_DOWNWARD_FACING

                    # Field of view (TFmini has a very narrow vertical field of view)
                    msg.h_fov = 0.0  # Horizontal field of view (not applicable)
                    msg.v_fov = math.radians(3.6)  # Vertical field of view in radians (~3.6 degrees)

                    # Quaternion for no rotation (default)
                    msg.q = [0.0, 0.0, 0.0, 1.0]  # Default quaternion

                    # Publish the message to PX4
                    self.publisher_.publish(msg)

                    # Throttled stats (no per-frame logs)
                    self._ok_frames += 1
                    now = time.time()
                    if now - self._last_log_sec > 5.0:
                        self.get_logger().info(f'TFmini ok={self._ok_frames} bad={self._bad_frames}')
                        self._ok_frames = 0
                        self._bad_frames = 0
                        self._last_log_sec = now

            except serial.SerialException as e:
                self.get_logger().error(f"Error reading from serial port: {str(e)}")
        else:
            # Do not spam warnings; just count as bad frame and log every ~5s
            self._bad_frames += 1
            now = time.time()
            if now - self._last_log_sec > 5.0:
                self.get_logger().info(f'TFmini ok={self._ok_frames} bad={self._bad_frames}')
                self._ok_frames = 0
                self._bad_frames = 0
                self._last_log_sec = now

def main(args=None):
    rclpy.init(args=args)
    node = TFminiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
