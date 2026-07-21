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
    # Measured on the rover 2026-07-21:
    #   cam_z  0.42  -- 18 cm chassis + 24 cm mast, ground to lens (user tape measure)
    #   cam_y  0.00  -- camera on the centreline of the 40.5 cm top plate
    #   pitch/roll 0 -- MEASURED, not assumed: camera IMU (/camera/accel/sample) read
    #                   gravity at 9.785 of 9.787 m/s2 on one axis, orthogonal
    #                   components -0.7 deg and +0.9 deg => level within ~1 deg.
    #
    #   cam_x -0.125 -- wheelbase 0.43 (front hub to rear hub), lens 0.34 behind the
    #                   front axle => 0.43/2 - 0.34. Camera sits 12.5 cm BEHIND the
    #                   rotation centre; negative is expected, not an error.
    #                   Cross-check: lens is 0.49 back from the front chassis edge and
    #                   the front axle is 0.15 back from it, so the lens is 0.34 behind
    #                   the front axle and 0.09 ahead of the rear -> 0.34+0.09=0.43. OK.
    mount_args = [
        DeclareLaunchArgument('cam_x', default_value='-0.125'),
        DeclareLaunchArgument('cam_y', default_value='0.00'),
        DeclareLaunchArgument('cam_z', default_value='0.42'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
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
            'scan_height': 40,
            'scan_time': 0.033,
            'range_min': 0.3,   # below the 336L's usable depth floor
            'range_max': 8.0,
            'output_frame': 'camera_depth_frame',
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription(mount_args + [base_to_camera, depth_to_scan])
