#!/usr/bin/env python3
"""Differential wheel odometry for the Vind-Roz rover.

Reads VESC ERPM per wheel from /fmu/out/esc_status (px4_msgs/EscStatus,
UAVCAN addresses 10-13), computes differential-drive odometry and publishes
nav_msgs/Odometry on /odom plus the odom -> base_link TF.

Wheel mapping, ERPM sign map, conversion constant and deadband are all
bench-verified values — see config/rover_odometry.yaml and
docs/rover_autonav_requirements.md (M1).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import EscStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('left_addresses', [11, 13])
        self.declare_parameter('right_addresses', [10, 12])
        self.declare_parameter('wheel_addresses', [10, 11, 12, 13])
        self.declare_parameter('wheel_signs', [-1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('erpm_to_ms', 0.000380)
        # MEASURED 2026-07-21: 0.31 m, left hub centre to right hub centre.
        # NOT 0.43 -- that is the WHEELBASE (front hub to rear hub); it sat in this
        # slot until now and under-reported every yaw rate by ~28% (0.31/0.43).
        self.declare_parameter('track_width', 0.31)
        self.declare_parameter('deadband_erpm', 40.0)
        self.declare_parameter('esc_timeout', 0.30)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.left_addrs = set(self.get_parameter('left_addresses').value)
        self.right_addrs = set(self.get_parameter('right_addresses').value)
        addrs = self.get_parameter('wheel_addresses').value
        signs = self.get_parameter('wheel_signs').value
        self.sign = dict(zip(addrs, signs))
        self.erpm_to_ms = self.get_parameter('erpm_to_ms').value
        self.track = self.get_parameter('track_width').value
        self.deadband = self.get_parameter('deadband_erpm').value
        self.esc_timeout_us = self.get_parameter('esc_timeout').value * 1e6
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # integrated state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_stamp_us = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            EscStatus, '/fmu/out/esc_status', self.esc_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            f'wheel odometry up: L={sorted(self.left_addrs)} R={sorted(self.right_addrs)} '
            f'signs={self.sign} track={self.track}m deadband=±{self.deadband} ERPM')

    def esc_callback(self, msg: EscStatus):
        reports = list(msg.esc[:msg.esc_count])

        # Nested esc[].timestamp stays on PX4's boot-relative clock — uXRCE-DDS
        # only offsets the top-level timestamp — so per-ESC staleness is measured
        # against the newest nested stamp, never against msg.timestamp.
        stamps = [r.timestamp for r in reports if r.timestamp > 0]
        if not stamps:
            self.get_logger().warning('no ESC timestamps — skipping update',
                                      throttle_duration_sec=5.0)
            self.prev_stamp_us = msg.timestamp
            return
        ref_us = max(stamps)

        left, right = [], []
        for i, r in enumerate(reports):
            if r.timestamp == 0 or r.esc_address not in self.sign:
                continue
            if not (msg.esc_online_flags >> i) & 1:
                continue
            if ref_us - r.timestamp > self.esc_timeout_us:
                continue
            erpm = float(r.esc_rpm)
            if abs(erpm) < self.deadband:
                erpm = 0.0
            v = erpm * self.sign[r.esc_address] * self.erpm_to_ms
            if r.esc_address in self.left_addrs:
                left.append(v)
            elif r.esc_address in self.right_addrs:
                right.append(v)

        if not left or not right:
            self.get_logger().warning(
                f'incomplete wheel data (L:{len(left)} R:{len(right)}) — skipping update',
                throttle_duration_sec=5.0)
            self.prev_stamp_us = msg.timestamp
            return

        v_left = sum(left) / len(left)
        v_right = sum(right) / len(right)
        v_lin = (v_left + v_right) / 2.0
        v_ang = (v_right - v_left) / self.track

        if self.prev_stamp_us is not None:
            dt = (msg.timestamp - self.prev_stamp_us) / 1e6
            if 0.0 < dt < 0.5:
                theta_mid = self.theta + v_ang * dt / 2.0
                self.x += v_lin * math.cos(theta_mid) * dt
                self.y += v_lin * math.sin(theta_mid) * dt
                self.theta = math.atan2(math.sin(self.theta + v_ang * dt),
                                        math.cos(self.theta + v_ang * dt))
        self.prev_stamp_us = msg.timestamp

        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_lin
        odom.twist.twist.angular.z = v_ang
        # x, y, yaw from wheel integration; z/roll/pitch unobserved
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = 0.005
        odom.twist.covariance[35] = 0.01
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
