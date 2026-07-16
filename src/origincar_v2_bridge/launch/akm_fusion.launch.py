"""
Ackermann Fusion Launch File
=============================
Launches the complete V2 inertial navigation stack:
  1. v2_serial_node      - Serial protocol bridge (V2 uplink + V1 downlink)
  2. imu_filter_madgwick - Raw IMU → filtered orientation
  3. nonholonomic_node   - Ackermann vy=0 constraint
  4. zupt_monitor        - Zero-velocity updates when static
  5. ekf_node            - EKF sensor fusion → /odometry/filtered
  6. static TFs          - base_link → imu_link
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('origincar_v2_bridge')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_akm.yaml')
    imu_config = os.path.join(pkg_share, 'config', 'imu_madgwick.yaml')

    # Declare arguments
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')

    # 1. V2 Serial Bridge
    v2_serial = Node(
        package='origincar_v2_bridge',
        executable='v2_serial_node',
        name='v2_serial_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': 115200,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'imu_frame_id': 'imu_link',
            'publish_tf': False,
        }],
    )

    # 2. Madgwick IMU Filter
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[imu_config],
    )

    # 3. Nonholonomic Constraint (vy=0)
    nonholonomic = Node(
        package='origincar_v2_bridge',
        executable='nonholonomic_node',
        name='nonholonomic_node',
        output='screen',
        parameters=[{
            'publish_rate': 50.0,
            'sigma_vy': 0.01,
        }],
    )

    # 4. ZUPT Monitor
    zupt = Node(
        package='origincar_v2_bridge',
        executable='zupt_monitor',
        name='zupt_monitor',
        output='screen',
        parameters=[{
            'sigma_v': 0.001,
            'sigma_omega': 0.001,
        }],
    )

    # 5. EKF Node
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom_combined')],
    )

    # 6. Static TF: base_link → imu_link (identity if IMU mounted flat)
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        v2_serial,
        imu_filter,
        nonholonomic,
        zupt,
        ekf,
        base_to_imu,
    ])
