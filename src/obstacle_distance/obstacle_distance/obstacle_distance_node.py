import rclpy
from rclpy.node import Node
from px4_msgs.msg import ObstacleDistance
import time
import VL53L1X

class ObstacleDistancePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_distance_publisher')

        # Initialize VL53L1X sensor
        self.sensor = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        self.sensor.open()
        self.sensor.start_ranging(3)  # Set to long distance mode

        # ROS 2 Publisher
        self.publisher_ = self.create_publisher(ObstacleDistance, '/fmu/in/obstacle_distance', 10)
        self.timer = self.create_timer(0.1, self.publish_obstacle_distance_data)
        self.system_start_time = time.time()

    def publish_obstacle_distance_data(self):
        msg = ObstacleDistance()

        # Get distance from the VL53L1X sensor
        distance_mm = self.sensor.get_distance()

        # Timestamp in microseconds since system start
        msg.timestamp = int((time.time() - self.system_start_time) * 1e6)

        # Set the frame of reference and sensor type
        msg.frame = ObstacleDistance.MAV_FRAME_BODY_FRD  # Body-fixed frame
        msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER  # Laser sensor

        # Fill in the distances array
        distances = [65535] * 72  # Initialize with no obstacle detected

        # Set front sensor data (for the front, index 0-5)
        if distance_mm < 4000:  # Check if the distance is within the sensor's range (4m)
            distance_cm = distance_mm // 10  # Convert mm to cm
            distances[0:6] = [distance_cm] * 6  # Front sensor covers index 0-5
        else:
            distances[0:6] = [65535] * 6  # If no valid data, mark as no obstacle

        # Set the distances array in the message
        msg.distances = distances

        # Increment of 5 degrees between each array element (since 72 elements cover 360 degrees)
        msg.increment = 5.0

        # Set angle offset and min/max distances
        msg.angle_offset = -13.5  # Front is at 0 degrees
        msg.min_distance = 20  # 20 cm minimum sensor range
        msg.max_distance = 400  # 400 cm maximum sensor range

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published front obstacle distance: {distance_mm} mm")

    def stop_sensor(self):
        self.sensor.stop_ranging()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDistancePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_sensor()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
