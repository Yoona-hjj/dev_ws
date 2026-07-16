"""
Waypoint Navigation Launch File
=================================
Launches:
  1. akm_fusion (V2 serial + Madgwick + EKF + ZUPT + nonholonomic)
  2. waypoint_nav_node (PID waypoint follower)

Usage:
  ros2 launch origincar_v2_bridge waypoint_nav.launch.py
  # Then send: ros2 topic pub /nav_command std_msgs/msg/String "data: 'start'" --once
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('origincar_v2_bridge')
    waypoints_config = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    # Include the fusion stack
    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'akm_fusion.launch.py')),
    )

    # Waypoint navigation node
    waypoint_nav = Node(
        package='origincar_v2_bridge',
        executable='waypoint_nav_node',
        name='waypoint_nav_node',
        output='screen',
        parameters=[waypoints_config],
    )

    return LaunchDescription([
        fusion_launch,
        waypoint_nav,
    ])
