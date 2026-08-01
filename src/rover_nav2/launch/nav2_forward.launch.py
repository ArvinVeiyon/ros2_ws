#!/usr/bin/env python3
"""Bring up Nav2 for FORWARD-ONLY autonomy on the depth camera alone.

Starts only the servers this scope needs. Notably absent:
  * map_server / amcl -- there is no map and no SLAM in this configuration;
    both costmaps run rolling in `odom`.
  * collision_monitor -- the reflex collision-stop already lives INSIDE the
    autonav_mode executor, which is the single funnel to the motors. Adding a
    second independent brake would make it ambiguous which one stopped the
    rover, and the executor's version cannot be bypassed by anything on
    /cmd_vel, which is a stronger guarantee than a Nav2 node can offer.

Assumes already running (systemd): rover-camera, rover-scan, rover-odometry,
rover-autonav-mode. rover-ekf-bridge must be started BY HAND and only with the
wheels on the floor.

    ros2 launch rover_nav2 nav2_forward.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rover_nav2')
    default_params = os.path.join(pkg, 'config', 'nav2_forward.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Order matters: the lifecycle manager brings these up in sequence, and the
    # controller needs its costmap configured before it will activate.
    managed = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    common = [{'use_sim_time': use_sim_time}, params_file]

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),

        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=common,
             # Nav2 1.3.12 emits geometry_msgs/Twist here and autonav_mode
             # subscribes to exactly that, so the chain needs no adapter. The
             # smoother sits between them and owns the final /cmd_vel.
             remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=common),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=common),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=common),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time},
                         {'autostart': autostart},
                         {'node_names': managed}]),
    ])
