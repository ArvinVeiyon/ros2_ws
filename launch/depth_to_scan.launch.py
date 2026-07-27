"""Convert the Gemini 336L depth image into a LaserScan on /scan (L4).

Assumes the Orbbec wrapper is already running (gemini_330_series.launch.py); it
publishes the camera-internal TF tree rooted at camera_link, so the only frame
this file adds is the physical mount: base_link -> camera_link.

    ros2 launch ~/ros2_ws/launch/depth_to_scan.launch.py

Standalone on purpose -- promote to a package once /scan is proven.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # MOUNT POSE -- metres from base_link origin, ROS convention: x forward, y left, z up.
    # base_link = ground level, midway between the axles, on the vehicle centreline.
    #
    # ---------------------------------------------------------------------
    # REMOUNT 2026-07-26 -- custom printed bracket, 70 mm above the top plate.
    # Values below are the DESIGN TARGET. They go live on the next
    # `systemctl restart rover-scan`. cam_z and pitch/roll are PENDING
    # verification against the built hardware -- do not treat as measured
    # until the as-built check is done (see "AFTER FITTING" at the bottom).
    #
    # Rover geometry (user tape measure 2026-07-26 -- SUPERSEDES 07-21):
    #   top plate        0.730 m long x 0.450 m wide   (07-21 said 0.405 wide)
    #   ground to plate  0.235 m                       (07-21 said 0.180 -- WRONG.
    #                    The 0.42 total was a direct measure to the lens, so the
    #                    old mast was really 0.185 above the plate, not 0.24.)
    #   front tip -> front axle   0.130 m              (07-21 said 0.150)
    #   wheelbase (axle to axle)  0.430 m
    #   track  (hub to hub)       0.310 m
    #   Plate decomposition check: 0.130 + 0.430 + 0.170 = 0.730 OK
    #
    #   cam_x  0.00  -- camera now sits ON the rotation centre, 0.345 m back from
    #                   the front tip (0.130 + 0.430/2). Was -0.125. Skid-steer
    #                   rotates about this point, so a centred sensor sees pure
    #                   rotation when spinning in place -- an off-centre sensor
    #                   traces an arc and injects fake translation into scan
    #                   matching. Centring also makes cam_x measurement error
    #                   nearly free. Bonus: the ~0.3 m sensor minimum range now
    #                   falls INSIDE the footprint (0.345 - 0.30 = 0.045 m behind
    #                   the bumper), so there is no blind strip ahead of the rover.
    #                   Mounting at the front tip would have opened a 0.30 m one.
    #   cam_y  0.00  -- centreline of the 0.450 m plate.
    #   cam_z  0.305 -- 0.235 plate + 0.070 bracket. Down from 0.42.
    #                   Deck-clearance floor is 0.017 m: the /scan band's lower
    #                   edge runs 2.79 deg below the axis (see scan_height note),
    #                   and the plate tip is 0.345 m ahead, so the lens must clear
    #                   0.345*tan(2.79) = 0.017 m or the rover's own deck shows up
    #                   in /scan as a permanent obstacle ~0.35 m ahead and the
    #                   reflex collision-stop never lets it move. 0.070 is ~4x that.
    #                   70 (not 55-60) because the USB exits downward and the plug
    #                   + cable occupy ~0.060 m: at 0.060 the plug would land on the
    #                   plate and the camera would rock on the connector instead of
    #                   sitting flat -- an unmeasurable pitch error, plus cable strain.
    #   pitch/roll   -- MEASURED 2026-07-27 after the remount, rover on flat floor,
    #                   mount flat. 4561 samples of /camera/accel/sample at rest:
    #                   mean (x -0.098, y -9.765, z -0.397), |g| 9.774 vs 9.81,
    #                   sd <= 0.34. The topic is camera_accel_OPTICAL_frame
    #                   (x right, y down, z fwd), so convert to camera_link
    #                   (x fwd, y left, z up) before reading angles off it:
    #                   x_link = z_opt, y_link = -x_opt, z_link = -y_opt.
    #                   Up-vector in link coords (-0.0406, +0.0100, +0.9992)
    #                   => pitch +0.0406 rad (2.33 deg NOSE DOWN),
    #                      roll  +0.0100 rad (0.57 deg, left side up).
    #                   Yaw is unobservable from gravity; left at 0.
    #                   The 07-21 values are VOID (measured before the remount).
    #                   Pitch matters more than it looks: the band is
    #                   cam_z +/- 0.049*d, and a downward pitch drops the centre
    #                   by tan(pitch)*d as well, so the lower edge is 0.0897*d
    #                   and bare floor enters /scan at 0.305/0.0897 = 3.4 m
    #                   instead of 6.25 m -- it reads as a wall dead ahead and
    #                   the reflex collision-stop will act on it. This is why the
    #                   angle is worth carrying rather than rounding to zero.
    mount_args = [
        DeclareLaunchArgument('cam_x', default_value='0.00'),
        DeclareLaunchArgument('cam_y', default_value='0.00'),
        DeclareLaunchArgument('cam_z', default_value='0.305'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0406'),
        DeclareLaunchArgument('cam_roll', default_value='0.0100'),
    ]

    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[{
            # Rows sampled around the image centre; the nearest hit in each
            # column wins, so a taller band sees low obstacles at the cost of
            # flagging the ground on any downward pitch.
            #
            # 40 rows is NOT a wide fan -- with the live depth intrinsics
            # (fx=fy=409.85, 848x480, so HFOV 91.9 deg / VFOV 60.7 deg) it is
            # +/- 20 px = +/- 2.79 deg. The /scan is a thin slab covering
            # heights cam_z +/- 0.049*d, i.e. cam_z CHOOSES WHICH HORIZONTAL
            # PLANE the rover senses. Consequences at cam_z 0.305:
            #   shortest obstacle seen   0.256 m @ 1 m, 0.207 m @ 2 m
            #   floor enters the band at 20.5*cam_z = 6.25 m
            # (At the old 0.42 it was 0.371 m @ 1 m -- blind to anything shorter
            # than the chassis is tall. That is why the mount came down.)
            # Do NOT "fix" low-obstacle blindness by raising scan_height: 120
            # rows would see 0.20 m at 1.5 m but pull floor detection in to
            # 2.9 m, forcing range_max down to ~2.5 m. The real answer is the
            # STL-19 owning /scan (roadmap O1).
            'scan_height': 40,
            'scan_time': 0.033,
            'range_min': 0.3,   # below the 336L's usable depth floor
            # 5.0, not 8.0: the floor enters the scan band at 6.25 m with
            # cam_z 0.305, and would be published as an obstacle. Indoors the
            # extra range was never used. Re-derive if cam_z changes.
            'range_max': 5.0,
            'output_frame': 'camera_depth_frame',
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription(mount_args + [base_to_camera, depth_to_scan])


# ---------------------------------------------------------------------------
# AFTER FITTING THE 2026-07-26 BRACKET -- do all four, none are optional:
#
# 1. Measure ground -> lens and set cam_z. Measure to the LEFT IR IMAGER, which
#    is the 336L's depth origin, NOT the housing centre -- getting this wrong
#    puts a silent ~20 mm offset in the whole map.
# 2. Re-derive pitch/roll from the camera's own IMU, do not eyeball it:
#      ros2 topic echo /camera/accel/sample --once
#    Gravity should sit on one axis; the orthogonal components give the tilt.
# 3. systemctl restart rover-scan   (the service launches this file with NO
#    argument overrides, so these defaults ARE the live TF)
# 4. Tape-measure one /scan return against a wall to confirm -- still
#    outstanding from the 07-24 checklist, only rate/plausibility proven so far.
#
# Known and accepted: at cam_z 0.305 with cam_x 0 the rover's own top plate
# fills roughly the bottom third of the depth frame (from 0.077 m ahead of the
# lens out to the plate tip at 0.345 m). Harmless for /scan, which reads only
# the centre band -- but the L5 Nav2 voxel layer consumes the full cloud and
# will need that self-return cropped.
# ---------------------------------------------------------------------------
