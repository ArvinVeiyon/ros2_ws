#!/usr/bin/env python3
"""Height-aware /scan_3d from the Gemini 336L point cloud.

WHY THIS EXISTS
---------------
`depthimage_to_laserscan` (the existing /scan) takes a band of image ROWS centred
on the optical axis. Rows are ANGLES, not heights, so the height band it actually
covers shrinks and drifts with distance. With the camera at 0.305 m pitched 2.3
deg down and scan_height=40, at 1 m ahead it only sees roughly 0.22-0.31 m above
the floor. A 15 cm box, a threshold, a cable or a shoe at 1 m is invisible to it
-- and those are exactly the obstacles a ground vehicle hits.

`pointcloud_to_laserscan` instead filters by TRUE HEIGHT in base_link, and emits
the nearest return at ANY height inside the band. That gives full 3D coverage of
the collision envelope at 2D cost, and the existing collision reflex can consume
it unchanged.

DELIBERATELY PUBLISHES /scan_3d, NOT /scan
------------------------------------------
/scan currently feeds the reflex that is the last line of defence before the
motors. It works. This runs ALONGSIDE it so the two can be compared on real
obstacles first; switching the reflex over is a separate, deliberate step once
/scan_3d is proven better rather than merely newer.

    ros2 launch ~/ros2_ws/launch/cloud_to_scan.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # Height band of the collision envelope, in base_link (z=0 at the floor).
        # min_height clears the floor itself: set it too low and every scan reads
        # "blocked" because the ground is an obstacle. VERIFY the floor really is
        # at z~0 in base_link before trusting these -- if base_link sits at axle
        # height the floor is negative and this band is wrong.
        DeclareLaunchArgument('min_height', default_value='0.06'),
        # Top of what the rover can collide with. Anything above this it drives
        # under, so treating it as an obstacle would refuse valid paths.
        DeclareLaunchArgument('max_height', default_value='0.45'),
        DeclareLaunchArgument('angle_min', default_value='-0.80'),   # rad, ~-46 deg
        DeclareLaunchArgument('angle_max', default_value='0.80'),    # rad, ~+46 deg
        DeclareLaunchArgument('range_min', default_value='0.20'),
        DeclareLaunchArgument('range_max', default_value='5.0'),
        DeclareLaunchArgument('cloud_topic', default_value='/camera/depth/points'),
    ]

    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='cloud_to_scan',
        remappings=[('cloud_in', LaunchConfiguration('cloud_topic')),
                    ('scan', '/scan_3d')],
        parameters=[{
            # base_link so the height filter means height above the floor, which
            # is the whole point. In camera_link it would tilt with the mount.
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': LaunchConfiguration('min_height'),
            'max_height': LaunchConfiguration('max_height'),
            'angle_min': LaunchConfiguration('angle_min'),
            'angle_max': LaunchConfiguration('angle_max'),
            'angle_increment': 0.0087,      # ~0.5 deg
            'scan_time': 0.05,
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
            # Keep the CLOSEST return per bearing. For a safety input this is the
            # only defensible choice -- averaging or taking the far return would
            # under-report exactly when it matters.
            'use_inf': True,
            'inf_epsilon': 1.0,
            # 0 = use all available cores. Pinned to 1: this box has 4 cores
            # already carrying camera, odometry and control, with a documented
            # starvation failure mode. Do not let a helper node eat the loop.
            'concurrency_level': 1,
        }],
        output='screen',
    )

    return LaunchDescription(args + [cloud_to_scan])
