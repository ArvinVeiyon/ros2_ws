#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry

class FakeVO(Node):
    def __init__(self):
        super().__init__('fake_visual_odometry')

        # PX4 listens here for external vision (your topic list shows it)
        self.pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            10
        )

        # 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.tick)

        # fixed pose at origin with slight altitude above ground (NED: down positive)
        self.x = 0.0
        self.y = 0.0
        self.z = -0.5  # 0.5 m above ground in NED
        self.get_logger().info("Publishing fake visual odometry to /fmu/in/vehicle_visual_odometry (30 Hz)")

        # variances (diagonal) — small nonzero values indicate confidence
        self.pos_var = 0.01
        self.ang_var = 0.01
        self.vel_var = 0.04

    def tick(self):
        m = VehicleOdometry()

        # timestamps: 0 is okay with uXRCE (PX4 will fill), or use now().nanoseconds if timesync is converged
        m.timestamp = 0
        m.timestamp_sample = 0

        # Frames: pose + velocity in NED
        m.pose_frame = VehicleOdometry.POSE_FRAME_NED
        m.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # position (meters, NED)
        m.position[0] = self.x
        m.position[1] = self.y
        m.position[2] = self.z

        # orientation quaternion (Hamilton) [w, x, y, z] — identity (yaw=0)
        m.q[0] = 1.0  # w
        m.q[1] = 0.0  # x
        m.q[2] = 0.0  # y
        m.q[3] = 0.0  # z

        # linear velocity (m/s, NED)
        m.velocity[0] = 0.0
        m.velocity[1] = 0.0
        m.velocity[2] = 0.0

        # angular velocity (rad/s) in body frame
        m.angular_velocity[0] = 0.0
        m.angular_velocity[1] = 0.0
        m.angular_velocity[2] = 0.0

        # variances (diagonal)
        m.position_variance[0] = self.pos_var
        m.position_variance[1] = self.pos_var
        m.position_variance[2] = self.pos_var

        m.orientation_variance[0] = self.ang_var
        m.orientation_variance[1] = self.ang_var
        m.orientation_variance[2] = self.ang_var

        m.velocity_variance[0] = self.vel_var
        m.velocity_variance[1] = self.vel_var
        m.velocity_variance[2] = self.vel_var

        # resets/ids — leave zero
        m.reset_counter = 0

        self.pub.publish(m)

def main():
    rclpy.init()
    node = FakeVO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
