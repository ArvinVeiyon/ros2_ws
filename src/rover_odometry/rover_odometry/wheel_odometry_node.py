#!/usr/bin/env python3
"""Differential wheel odometry for the Vind-Roz rover.

Reads VESC ERPM per wheel from /fmu/out/esc_status (px4_msgs/EscStatus,
UAVCAN addresses 10-13), computes differential-drive odometry and publishes
nav_msgs/Odometry on /odom plus the odom -> base_link TF.

Wheel mapping, ERPM sign map, conversion constant and deadband are all
bench-verified values — see config/rover_odometry.yaml and
docs/rover_autonav_requirements.md (M1).

HEADING COMES FROM A GYRO, NOT THE WHEELS (yaw_source, default 'camera_gyro').
This vehicle is skid-steer: it has no steering axle and can only rotate by
forcing all four tyres to scrub sideways across the ground. Slip is therefore
not a defect to be minimised, it is the turning mechanism — which means
(v_right - v_left) / track is measuring a quantity the wheels cannot observe,
and it over-reports rotation by an amount that varies with surface, load and
turn radius. Getting track_width exactly right (0.31, measured 2026-07-21)
removes a constant scale error but cannot touch the variable slip error.

A gyro senses rotation directly and does not care whether a wheel slipped, so
wheels are used for distance and a gyro for heading. That split is standard
practice for skid-steer, and it matters most for SLAM, which aligns scans by
pose: a wrong heading lands every scan rotated and smears the map.

⛔ WHY THE DEFAULT IS THE CAMERA, NOT THE FLIGHT CONTROLLER (measured 2026-08-09)
Both were compared against an absolute reference: park square to a flat wall and
RANSAC-fit a line to /scan, which gives perpendicular distance to +/-1 mm and wall
bearing to +/-0.26 deg. With the rover VERIFIED STATIONARY by that wall:

    window              wall (truth)   FC yaw      camera gyro
    parked, cold 476 s    -0.07 deg    -2.46 deg     -
    after driving 111 s   +0.01 deg   -18.88 deg     -
    after turning  21 s   -0.18 deg   +23.23 deg     -
    after turning  41 s   -0.07 deg    -7.17 deg    -0.02 deg

The FC invented +27.1 deg of rotation across one 4-minute session while the rover
never moved. It is ERRATIC rather than biased — the sign flips — and it is NOT
proportional to motor effort (0.341 m/s gave +0.54 deg of phantom yaw where
0.364 m/s gave -8.28 deg), so it cannot be calibrated or compensated. Root cause
unknown; EKF2_MAG_TYPE=1 so the magnetometer is in use, but the disable test was
never run and the fault is too intermittent for a single run to prove anything.

The 336L gyro over the same transition: bias +0.72 deg/s but stable to
0.0015 deg/s, and driving does not disturb it. Over a 263 s mapping run that is
~1 deg of heading error against tens to hundreds from the FC.

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
  * The camera gyro is a RATE sensor with a large bias, so it is integrated here
    rather than differenced like the FC's absolute attitude. Bias is re-estimated
    continuously while the wheels read zero, because it creeps thermally
    (+0.0083 deg/s over 3 minutes measured) — calibrating once and trusting it
    forever would reintroduce exactly the drift we are escaping.
  * Integration uses header.stamp deltas, NOT arrival time. A probe that used
    arrival time under-read rotation by an amount that grew with turn rate
    (-0.38% at 8.3 deg/s, -4.53% at 32.7 deg/s), which is the signature of
    samples being dropped at peak CPU and the next sample's lower rate being
    applied across the whole gap. Gaps are counted and reported instead.
  * The camera clock is its own domain (stamps are device uptime, not wall
    clock), so we also track stamp-elapsed vs wall-elapsed and warn if they
    diverge — a device clock running at the wrong rate would silently scale
    every integrated angle.
  * ⚠️ This couples odometry to rover-camera, which is a component that DEGRADES
    SILENTLY (colour rate decays over ~1.5 h; see memory [SERVICES]). Staleness
    falls back to the FC gyro and then to wheels, loudly.
  * If the selected source goes stale we fall back automatically rather than
    freezing heading, and say so in the log.
  * These VESCs doze at rest: after a few seconds without motion only address 13
    keeps reporting (esc_online_flags 8), so one side has no data and a naive
    reader concludes the wheels are unreadable. They are not — a dozing ESC
    cannot be driving its wheel, so if every ESC that is still awake reads zero
    the rover is provably stationary. We publish that as a zero-velocity
    measurement (publish_at_rest) rather than going silent, because /odom
    dropping out whenever the rover stops would break Nav2 and starve
    rover_ekf_bridge's EV aiding mid-mission.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import EscStatus, VehicleAttitude
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('left_addresses', [11, 13])
        self.declare_parameter('right_addresses', [10, 12])
        self.declare_parameter('wheel_addresses', [10, 11, 12, 13])
        self.declare_parameter('wheel_signs', [-1.0, 1.0, 1.0, 1.0])
        # 0.004633 -> 0.003900 on 2026-08-09 (divide by 1.188). The old value was
        # the slip-free ERPM->wheel-rotation figure from a hand push; odometry needs
        # ERPM->GROUND distance, and powered runs slip by a measured 18.8% (5 wall-
        # referenced runs, sd 2.4%). The uncorrected value drew every wall several
        # times in house_map_v3. See config/rover_odometry.yaml for the full record.
        self.declare_parameter('erpm_to_ms', 0.003900)
        # MEASURED 2026-07-21: 0.31 m, left hub centre to right hub centre.
        # NOT 0.43 -- that is the WHEELBASE (front hub to rear hub); it sat in this
        # slot until now and under-reported every yaw rate by ~28% (0.31/0.43).
        self.declare_parameter('track_width', 0.31)
        self.declare_parameter('deadband_erpm', 5.0)
        self.declare_parameter('esc_timeout', 0.30)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        # 'gyro' = heading from FC attitude (slip-immune, strongly preferred on a
        # skid-steer). 'wheels' = legacy (v_right-v_left)/track, kept for A/B
        # comparison and as a diagnostic if the FC link is down.
        self.declare_parameter('yaw_source', 'camera_gyro')
        self.declare_parameter('attitude_timeout', 0.5)
        # --- camera-gyro heading (the default source; see the module docstring) ---
        self.declare_parameter('camera_gyro_topic', '/camera/gyro/sample')
        self.declare_parameter('camera_gyro_frame', 'camera_gyro_optical_frame')
        # Staleness. At 195 Hz, 0.3 s is ~58 missed samples: unambiguously broken,
        # while still tolerating an ordinary scheduling hiccup.
        self.declare_parameter('camera_gyro_timeout', 0.3)
        # A gap larger than this between consecutive stamps is counted and logged.
        # Nominal spacing is 5.1 ms; 50 ms is ~10 missed samples.
        self.declare_parameter('camera_gyro_max_gap', 0.05)
        # Rolling window of at-rest samples used for the bias estimate. 12 s at
        # 195 Hz is ~2300 samples, which matched the offline characterisation.
        self.declare_parameter('camera_bias_window', 12.0)
        # Refuse to integrate until the bias is backed by at least this many
        # at-rest samples — an unbiased start would inject 0.72 deg/s of error.
        self.declare_parameter('camera_bias_min_samples', 400)
        # Keep publishing a zero-velocity /odom when dozing ESCs leave a side
        # unreported but every awake wheel reads zero. Set false to restore the
        # old behaviour (skip the update, /odom goes silent) for A/B testing.
        self.declare_parameter('publish_at_rest', True)

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
        self.publish_at_rest = self.get_parameter('publish_at_rest').value
        if self.yaw_source not in ('camera_gyro', 'gyro', 'wheels'):
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
        self.at_rest_active = False    # publishing inferred zero velocity?
        self.heading_src = None        # last announced source, for log-on-change

        # camera-gyro state
        self.cam_topic = str(self.get_parameter('camera_gyro_topic').value)
        self.cam_frame = str(self.get_parameter('camera_gyro_frame').value)
        self.cam_timeout = float(self.get_parameter('camera_gyro_timeout').value)
        self.cam_max_gap = float(self.get_parameter('camera_gyro_max_gap').value)
        self.cam_bias_window = float(self.get_parameter('camera_bias_window').value)
        self.cam_bias_min = int(self.get_parameter('camera_bias_min_samples').value)
        self.cam_row = None            # 3rd row of base_link <- camera rotation
        self.cam_yaw = 0.0             # integrated, unwrapped, bias-removed [rad]
        self.prev_cam_yaw = None       # value at the previous odometry step
        self.cam_prev_stamp = None     # previous header.stamp [s]
        self.cam_wall = None           # arrival time, for staleness
        self.cam_bias = None           # [rad/s]
        self.cam_bias_buf = deque()    # (stamp, wz) while the wheels are stopped
        self.cam_bias_sum = 0.0        # running sum, so the mean stays O(1)
        self.cam_gaps = 0
        self.cam_gap_logged = 0.0
        self.wheels_stopped = True     # set by esc_callback; gates bias updates
        # clock-rate sanity: camera stamps are device uptime, not wall clock
        self.cam_span_stamp = 0.0
        self.cam_span_wall = 0.0
        self.cam_clock_warned = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        # The Orbbec wrapper publishes IMU on sensor QoS (best-effort). Depth 50
        # covers a scheduling hiccup at 195 Hz without hoarding stale samples.
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.cam_sub = self.create_subscription(
            Imu, self.cam_topic, self.camera_gyro_callback, cam_qos)
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

    def _camera_row(self):
        """Third row of the base_link <- camera_gyro rotation, cached.

        Only the row is needed: yaw rate about base_link +z is its dot product
        with the camera-frame angular velocity. Taken from TF rather than
        hand-derived, so a remount cannot silently invert the sign.
        """
        if self.cam_row is not None:
            return self.cam_row
        try:
            tf = self.tf_buffer.lookup_transform(
                self.child_frame_id, self.cam_frame, rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        self.cam_row = (2.0 * (x * z - y * w),
                        2.0 * (y * z + x * w),
                        1.0 - 2.0 * (x * x + y * y))
        self.get_logger().info(
            f'camera gyro TF locked: {self.child_frame_id} <- {self.cam_frame} '
            f'row_z=({self.cam_row[0]:+.4f}, {self.cam_row[1]:+.4f}, {self.cam_row[2]:+.4f})')
        return self.cam_row

    def camera_gyro_callback(self, msg: Imu):
        row = self._camera_row()
        if row is None:
            return
        wz = (row[0] * msg.angular_velocity.x
              + row[1] * msg.angular_velocity.y
              + row[2] * msg.angular_velocity.z)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall = self.get_clock().now().nanoseconds / 1e9
        self.cam_wall = wall

        # Bias: rolling mean over the at-rest window. A dozing-ESC standstill is
        # still a standstill, so wheels_stopped covers it.
        if self.wheels_stopped:
            # Running sum over a deque: O(1) per sample rather than re-summing a
            # ~2300-element window 195 times a second.
            # ⚠️ MEASURED: this is NOT where the CPU goes. Steady-state cost went
            # 58.8% -> 53.5% with this change; the node was ~25% before the camera
            # gyro existed. The ~28-point jump is rclpy's per-message overhead at
            # 195 Hz, not the arithmetic in here (cf. the C++-port note: ~65% of
            # this node is executor overhead, ~4% is our logic). If this needs to
            # come down, the levers are a lower IMU publish rate at the source or
            # the C++ port — not micro-optimising this callback.
            self.cam_bias_buf.append((stamp, wz))
            self.cam_bias_sum += wz
            cutoff = stamp - self.cam_bias_window
            while self.cam_bias_buf and self.cam_bias_buf[0][0] < cutoff:
                self.cam_bias_sum -= self.cam_bias_buf.popleft()[1]
            if len(self.cam_bias_buf) >= self.cam_bias_min:
                was_none = self.cam_bias is None
                self.cam_bias = self.cam_bias_sum / len(self.cam_bias_buf)
                if was_none:
                    self.get_logger().info(
                        f'camera gyro bias established: {math.degrees(self.cam_bias):+.4f} '
                        f'deg/s from {len(self.cam_bias_buf)} at-rest samples')

        prev = self.cam_prev_stamp
        self.cam_prev_stamp = stamp
        if prev is None or self.cam_bias is None:
            return
        dt = stamp - prev
        if dt <= 0.0:
            return
        if dt > self.cam_max_gap:
            # Do NOT integrate across the gap: the rate we hold is the one AFTER
            # it, and applying that across the whole interval is what makes a
            # dropped burst read as lost rotation.
            self.cam_gaps += 1
            if wall - self.cam_gap_logged > 10.0:
                self.get_logger().warning(
                    f'camera gyro gap {dt*1000:.0f} ms (>{self.cam_max_gap*1000:.0f} ms); '
                    f'{self.cam_gaps} so far — rotation across gaps is NOT integrated')
                self.cam_gap_logged = wall
            return
        self.cam_yaw += (wz - self.cam_bias) * dt

        # Device clock vs wall clock. A camera clock running at the wrong rate
        # would scale every integrated angle without any other symptom.
        self.cam_span_stamp += dt
        self.cam_span_wall += min(max(wall - getattr(self, '_cam_prev_wall', wall), 0.0), 1.0)
        self._cam_prev_wall = wall
        if (not self.cam_clock_warned and self.cam_span_wall > 60.0
                and self.cam_span_stamp > 0.0):
            ratio = self.cam_span_stamp / self.cam_span_wall
            if abs(ratio - 1.0) > 0.01:
                self.cam_clock_warned = True
                self.get_logger().warning(
                    f'camera stamp clock runs at {ratio:.4f}x wall clock — every '
                    'integrated angle is scaled by this factor')

    def _camera_fresh(self):
        if self.cam_wall is None or self.cam_bias is None or self.cam_row is None:
            return False
        age = self.get_clock().now().nanoseconds / 1e9 - self.cam_wall
        return age <= self.cam_timeout

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

        left, right, online = [], [], []
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
            online.append(v)
            if r.esc_address in self.left_addrs:
                left.append(v)
            elif r.esc_address in self.right_addrs:
                right.append(v)

        at_rest = False
        if left and right:
            v_left = sum(left) / len(left)
            v_right = sum(right) / len(right)
        elif self.publish_at_rest and online and not any(v != 0.0 for v in online):
            # A side has no online ESC. At rest that is normal, not a dropout:
            # these VESCs doze after a few seconds of no motion and only addr 13
            # keeps reporting (esc_online_flags 8), which reads as L:1 R:0.
            #
            # We can still assert the rover is stationary, because a dozing ESC
            # cannot be driving its wheel — an ESC that is turning a motor is by
            # definition awake and reporting, and a wheel turned externally wakes
            # its ESC too (nudge test, 2026-07-26). So if every ESC that IS awake
            # reads inside the deadband, no wheel is moving. That makes this a
            # real zero-velocity measurement, not absent data, and publishing it
            # is what keeps /odom alive for Nav2 and rover_ekf_bridge.
            #
            # The guard is `any(...)`, so a single awake wheel reporting motion
            # while its opposite side is missing still falls through to the skip
            # below — a genuinely unreadable rover never gets reported as stopped.
            v_left = v_right = 0.0
            at_rest = True
        else:
            self.get_logger().warning(
                f'incomplete wheel data (L:{len(left)} R:{len(right)}) — skipping update',
                throttle_duration_sec=5.0)
            self.prev_stamp_us = msg.timestamp
            return

        if at_rest != self.at_rest_active:
            if at_rest:
                self.get_logger().info(
                    f'ESCs dozing (online_flags={msg.esc_online_flags}) and all awake '
                    'wheels at zero — publishing /odom at rest')
            else:
                self.get_logger().info('all wheels reporting again — /odom fully measured')
            self.at_rest_active = at_rest
        v_lin = (v_left + v_right) / 2.0
        v_ang_wheels = (v_right - v_left) / self.track

        # Tell the camera-gyro callback whether it may fold samples into the bias
        # estimate. at_rest (dozing ESCs, every awake wheel zero) is a genuine
        # standstill, so it counts.
        self.wheels_stopped = at_rest or (v_left == 0.0 and v_right == 0.0)

        # Source selection, in preference order. Each falls through to the next
        # when its sensor is stale, so heading degrades rather than freezing.
        source = 'wheels'
        if self.yaw_source == 'camera_gyro' and self._camera_fresh():
            source = 'camera_gyro'
        elif self.yaw_source in ('camera_gyro', 'gyro') and self._attitude_fresh():
            source = 'gyro'
        if source != self.heading_src:
            if source == 'camera_gyro':
                self.get_logger().info('heading source: CAMERA GYRO (336L, bias-corrected)')
            elif source == 'gyro':
                if self.yaw_source == 'camera_gyro':
                    self.get_logger().warning(
                        'camera gyro unavailable — falling back to FC attitude, which '
                        'was measured inventing up to 23 deg in 21 s while stationary')
                else:
                    self.get_logger().info('heading source: GYRO (FC attitude)')
            elif self.yaw_source == 'wheels':
                self.get_logger().info('heading source: WHEELS (configured)')
            elif self.heading_src is None:
                # First decision at startup: the sensor simply has not arrived
                # yet. Not a fault, so don't raise a warning for it.
                self.get_logger().info('waiting for a gyro; wheel yaw meanwhile')
            else:
                self.get_logger().warning(
                    'all gyro sources stale — falling back to wheel-derived yaw '
                    '(a skid-steer turns by scrubbing, so the wheels cannot '
                    'observe rotation at all)')
            self.heading_src = source
        self.gyro_active = source in ('camera_gyro', 'gyro')

        v_ang = v_ang_wheels
        if self.prev_stamp_us is not None:
            dt = (msg.timestamp - self.prev_stamp_us) / 1e6
            if 0.0 < dt < 0.5:
                if source == 'camera_gyro' and self.prev_cam_yaw is not None:
                    # Already in ROS convention (rotated into base_link via TF)
                    # and already unwrapped, so difference it directly.
                    d_theta = self.cam_yaw - self.prev_cam_yaw
                    v_ang = d_theta / dt
                elif source == 'gyro' and self.prev_att_yaw is not None:
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
        # Always advance the yaw baselines, even on a skipped step, so a dropped
        # or out-of-range dt never turns into a false accumulated rotation.
        if self.att_yaw is not None:
            self.prev_att_yaw = self.att_yaw
        self.prev_cam_yaw = self.cam_yaw

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
        # depends on its source, and the three differ by orders of magnitude, so
        # advertise that honestly — Nav2/SLAM weight poses by these numbers.
        #   camera gyro : drift 0.0015-0.0028 deg/s, unaffected by driving
        #   FC attitude : erratic, measured up to 1.106 deg/s while stationary
        #   wheels      : cannot observe rotation on a skid-steer at all
        yaw_var = {'camera_gyro': 0.002, 'gyro': 0.02, 'wheels': 0.2}[source]
        twist_var = {'camera_gyro': 0.001, 'gyro': 0.01, 'wheels': 0.1}[source]
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = yaw_var
        odom.twist.covariance[0] = 0.005
        odom.twist.covariance[35] = twist_var
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
