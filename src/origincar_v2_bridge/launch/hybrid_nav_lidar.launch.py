import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bridge = get_package_share_directory('origincar_v2_bridge')

    port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/wheeltec_lidar')
    frame_arg = DeclareLaunchArgument('lidar_frame_id', default_value='laser')
    scan_topic_arg = DeclareLaunchArgument('lidar_scan_topic', default_value='/scan')
    stm_port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                                          description='STM32 serial port device')

    hybrid_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'hybrid_nav.launch.py')),
        launch_arguments={
            'enable_lidar_avoid': 'false',
            'port': LaunchConfiguration('port'),
        }.items(),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'lslidar_uart.launch.py')),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
            'lidar_scan_topic': LaunchConfiguration('lidar_scan_topic'),
        }.items(),
    )

    # Delay LiDAR start by 3s to avoid serial port contention at boot
    lidar_delayed = TimerAction(period=3.0, actions=[lidar])

    return LaunchDescription([
        port_arg,
        frame_arg,
        scan_topic_arg,
        stm_port_arg,
        hybrid_nav,
        # lidar_delayed,
    ])
