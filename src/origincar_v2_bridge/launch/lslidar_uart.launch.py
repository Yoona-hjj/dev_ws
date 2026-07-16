import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bridge = get_package_share_directory('origincar_v2_bridge')
    default_config = os.path.join(pkg_bridge, 'config', 'lslidar_lsn10.yaml')

    config_arg = DeclareLaunchArgument('lidar_config', default_value=default_config)
    port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/wheeltec_lidar')
    frame_arg = DeclareLaunchArgument('lidar_frame_id', default_value='laser')
    scan_topic_arg = DeclareLaunchArgument('lidar_scan_topic', default_value='/scan')

    lidar_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('lidar_config'),
            {
                'serial_port_': LaunchConfiguration('lidar_port'),
                'frame_id': LaunchConfiguration('lidar_frame_id'),
                'scan_topic': LaunchConfiguration('lidar_scan_topic'),
            },
        ],
    )

    return LaunchDescription([
        config_arg,
        port_arg,
        frame_arg,
        scan_topic_arg,
        lidar_node,
    ])
