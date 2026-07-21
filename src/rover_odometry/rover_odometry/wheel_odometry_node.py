#!/usr/bin/env python3
"""Differential wheel odometry for the Vind-Roz rover.

Reads VESC ERPM per wheel from /fmu/out/esc_status (px4_msgs/EscStatus,
UAVCAN addresses 10-13), computes differential-drive odometry and publishes
nav_msgs/Odometry on /odom plus the odom -> base_link TF.

Wheel mapping, ERPM sign map, conversion constant and deadband are all
bench-verified values — see config/rover_odometry.yaml and
docs/rover_autonav_requirements.md (M1).

HEADING COMES FROM THE GYRO, NOT THE WHEELS (yaw_source, default 'gyro').
This vehicle is skid-steer: it has no steering axle and can only rotate by
forcing all four tyres to scrub sideways across the ground. Slip is therefore
not a defect to be minimised, it is the turning mechanism — which means
(v_right - v_left) / track is measuring a quantity the wheels cannot observe,
and it over-reports rotation by an amount that varies with surface, load and
turn radius. Getting track_width exactly right (0.31, measured 2026-07-21)
removes a constant scale error but cannot touch the variable slip error.

The FC's EKF-fused attitude senses rotation directly and does not care whether
a wheel slipped, so wheels are used for distance and the gyro for heading. That
split is standard practice for skid-steer, and it matters most for SLAM, which
aligns scans by pose: a wrong heading lands every scan rotated and smears the map.

Implementation notes:
  * We integrate yaw *deltas* rather than adopting PX4's absolute yaw, so /odom
    keeps its own origin (theta starts at 0 at node start) and we stay clear of
    NED-vs-ENU absolute frame conventions. Sign is flipped once: PX4 yaw is NED
    (z down, positive clockwise from above), ROS is ENU/FLU (z up, positive CCW).
  * quat_reset_counter changes are EKF resets — a step in yaw that the vehicle
    did not physically perform. The delta across a reset is dropped, otherwise
    the jump would be integrated as real rotation and corrupt odom continuity.
  * /fmu/out/vehicle_angular_velocity is NOT in this FC's dds_topics.yaml, so
    vehicle_attitude (~92 Hz, measured yaw noise 0.049 deg over 8 s at rest) is
    the gyro-derived source available to us.
  * If attitude goes stale we fall back to wheel-derived yaw automatically
    rather than freezing heading, and say so in the log.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import EscStatus, VehicleAttitude
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
        # 'gyro' = heading from FC attitude (slip-immune, strongly preferred on a
        # skid-steer). 'wheels' = legacy (v_right-v_left)/track, kept for A/B
        # comparison and as a diagnostic if the FC link is down.
        self.declare_parameter('yaw_source', 'gyro')
        self.declare_parameter('attitude_timeout', 0.5)

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
        self.yaw_source = str(self.get_parameter('yaw_source').value).lower()
        self.attitude_timeout = self.get_parameter('attitude_timeout').value
        if self.yaw_source not in ('gyro', 'wheels'):
            self.get_logger().warning(
                f"unknown yaw_source '{self.yaw_source}' — falling back to 'wheels'")
            self.yaw_source = 'wheels'

        # integrated state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_stamp_us = None

        # attitude state (gyro heading)
        self.att_yaw = None            # latest PX4 yaw, NED, rad
        self.att_wall = None           # arrival time, for staleness
        self.att_reset_count = None
        self.prev_att_yaw = None       # yaw at the previous odometry step
        self.gyro_active = None        # None until first decision, then bool

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            EscStatus, '/fmu/out/esc_status', self.esc_callback, qos)
        self.att_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            f'wheel odometry up: L={sorted(self.left_addrs)} R={sorted(self.right_addrs)} '
            f'signs={self.sign} track={self.track}m deadband=±{self.deadband} ERPM '
            f'yaw_source={self.yaw_source}')

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def attitude_callback(self, msg: VehicleAttitude):
        w, x, y, z = (float(v) for v in msg.q)
        # PX4 q is the NED->FRD rotation; yaw about the down axis.
        self.att_yaw = math.atan2(2.0 * (w * z + x * y),
                                  1.0 - 2.0 * (y * y + z * z))
        self.att_wall = self.get_clock().now().nanoseconds / 1e9
        if self.att_reset_count is None:
            self.att_reset_count = msg.quat_reset_counter
        elif msg.quat_reset_counter != self.att_reset_count:
            # EKF reset: yaw stepped without the vehicle turning. Re-baseline so
            # the jump is never integrated as rotation.
            self.att_reset_count = msg.quat_reset_counter
            self.prev_att_yaw = self.att_yaw
            self.get_logger().info('EKF yaw reset — delta dropped, heading rebaselined')

    def _attitude_fresh(self):
        if self.att_yaw is None or self.att_wall is None:
            return False
        age = self.get_clock().now().nanoseconds / 1e9 - self.att_wall
        return age <= self.attitude_timeout

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
        v_ang_wheels = (v_right - v_left) / self.track

        use_gyro = self.yaw_source == 'gyro' and self._attitude_fresh()
        if use_gyro != self.gyro_active:
            if use_gyro:
                self.get_logger().info('heading source: GYRO (FC attitude)')
            elif self.yaw_source == 'wheels':
                self.get_logger().info('heading source: WHEELS (configured)')
            elif self.gyro_active is None:
                # First decision at startup: attitude simply has not arrived yet.
                # Not a fault, so don't raise a warning for it.
                self.get_logger().info('waiting for FC attitude; wheel yaw meanwhile')
            else:
                self.get_logger().warning(
                    'FC attitude stale — falling back to wheel-derived yaw '
                    '(slip-prone on skid-steer)')
            self.gyro_active = use_gyro

        v_ang = v_ang_wheels
        if self.prev_stamp_us is not None:
            dt = (msg.timestamp - self.prev_stamp_us) / 1e6
            if 0.0 < dt < 0.5:
                if use_gyro and self.prev_att_yaw is not None:
                    # NED -> ENU: PX4 yaw is positive clockwise seen from above,
                    # ROS is positive counter-clockwise, hence the negation.
                    d_theta = -self._wrap(self.att_yaw - self.prev_att_yaw)
                    v_ang = d_theta / dt
                else:
                    d_theta = v_ang_wheels * dt
                theta_mid = self.theta + d_theta / 2.0
                self.x += v_lin * math.cos(theta_mid) * dt
                self.y += v_lin * math.sin(theta_mid) * dt
                self.theta = self._wrap(self.theta + d_theta)
        self.prev_stamp_us = msg.timestamp
        # Always advance the yaw baseline, even on a skipped step, so a dropped
        # or out-of-range dt never turns into a false accumulated rotation.
        if self.att_yaw is not None:
            self.prev_att_yaw = self.att_yaw

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
        # x, y from wheel integration; z/roll/pitch unobserved. Yaw confidence
        # depends on its source: the gyro is far better than slipping wheels, so
        # advertise that honestly — Nav2/SLAM weight poses by these numbers.
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.002 if use_gyro else 0.02
        odom.twist.covariance[0] = 0.005
        odom.twist.covariance[35] = 0.001 if use_gyro else 0.01
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
