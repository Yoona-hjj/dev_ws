"""
Hybrid Navigation Launch File
===============================
Runs odometry-based waypoint following first, then hands control to the
vision line-following / obstacle-avoidance / P-point parking stack at the
X-crossing return leg (where accumulated IMU drift makes odometry unreliable).

Pipeline:
  1. Camera:   hobot_usb_cam -> /hbmem_img -> hobot_codec_decode -> /nv12_img
  2. Odometry: akm_fusion (V2 serial + Madgwick + EKF + ZUPT) -> /odom_combined
  3. waypoint_nav_node: follows waypoints, publishes /cmd_vel.
       On reaching `vision_handoff_wp_idx` it stops driving and publishes
       /car_go = -10 to activate the vision stack.
  4. Vision:   racing_track_detection (line center)
               racing_obstacle_detection (obstacles + P)
               racing_control -> /racing
               control_master  -> /cmd_vel  (silent until /car_go = -10)

Usage:
  ros2 launch origincar_v2_bridge hybrid_nav.launch.py
  ros2 topic pub /nav_command std_msgs/msg/String "data: 'start'" --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bridge = get_package_share_directory('origincar_v2_bridge')
    waypoints_config = os.path.join(pkg_bridge, 'config', 'waypoints.yaml')

    handoff_arg = DeclareLaunchArgument(
        'vision_handoff_wp_idx', default_value='420',
        description='Waypoint index at which to hand control to the vision stack (final leg)')
    lead_arg = DeclareLaunchArgument(
        'lead_with_vision', default_value='true',
        description='Drive with vision from start until QR is scanned at the teardrop')
    resume_arg = DeclareLaunchArgument(
        'odom_resume_wp_idx', default_value='118',
        description='Waypoint index to resume odometry following after QR scan (teardrop tip)')
    route_branch_arg = DeclareLaunchArgument(
        'route_branch_wp_idx', default_value='175',
        description='Waypoint index at the middle crossing where QR-selected route branches')
    route_merge_arg = DeclareLaunchArgument(
        'route_merge_wp_idx', default_value='425',
        description='Waypoint index at the middle crossing where routes merge back to P')
    clockwise_vlm_wp_arg = DeclareLaunchArgument(
        'clockwise_vlm_wp_idx', default_value='329',
        description='Clockwise route waypoint before the upper-right corner')
    counterclockwise_vlm_wp_arg = DeclareLaunchArgument(
        'counterclockwise_vlm_wp_idx', default_value='325',
        description='Counterclockwise route waypoint before the upper-left corner')
    vlm_stop_arg = DeclareLaunchArgument('vlm_stop_sec', default_value='2.0')
    vlm_capture_delay_arg = DeclareLaunchArgument('vlm_capture_delay_sec', default_value='1.0')
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyACM0',
        description='STM32 serial port device')
    device_arg = DeclareLaunchArgument(
        'device', default_value='/dev/video0', description='USB camera device')
    end_y_p_arg = DeclareLaunchArgument('end_y_p', default_value='435')
    p_target_x_arg = DeclareLaunchArgument('p_target_x', default_value='290.0')
    p_align_tolerance_arg = DeclareLaunchArgument('p_align_tolerance', default_value='40.0')
    p_approach_x_arg = DeclareLaunchArgument('p_approach_x', default_value='0.40')
    p_kp_arg = DeclareLaunchArgument('p_kp', default_value='0.006')
    p_max_angular_arg = DeclareLaunchArgument('p_max_angular', default_value='1.2')
    p_force_stop_y_arg = DeclareLaunchArgument('p_force_stop_y', default_value='470')
    p_blind_start_y_arg = DeclareLaunchArgument('p_blind_start_y', default_value='330')
    p_blind_x_arg = DeclareLaunchArgument('p_blind_x', default_value='0.30')
    p_blind_duration_arg = DeclareLaunchArgument('p_blind_duration', default_value='0.8')
    qr_yaw_arg = DeclareLaunchArgument(
        'qr_turnaround_yaw_deg', default_value='70.0',
        description='Total heading rotation (deg) for the post-QR reverse+right-turn; smaller = shorter reverse')
    qr_reverse_speed_arg = DeclareLaunchArgument(
        'qr_turnaround_reverse_speed', default_value='0.30',
        description='Reverse speed during post-QR turnaround; lower = less inertia/overshoot')
    qr_angular_arg = DeclareLaunchArgument(
        'qr_turnaround_angular', default_value='3.5',
        description='Right-turn angular rate during post-QR turnaround; limited by steering max angle')
    qr_presteer_arg = DeclareLaunchArgument('qr_turnaround_presteer_sec', default_value='0.10')
    qr_slow_y_min_arg = DeclareLaunchArgument('qr_slow_y_min', default_value='10')
    qr_slow_y_max_arg = DeclareLaunchArgument('qr_slow_y_max', default_value='80')
    qr_stop_y_arg = DeclareLaunchArgument('qr_stop_y', default_value='130')
    qr_slow_x_arg = DeclareLaunchArgument('qr_slow_x', default_value='0.10')
    qr_bottom_timeout_arg = DeclareLaunchArgument('qr_bottom_timeout', default_value='0.8')
    enable_lidar_avoid_arg = DeclareLaunchArgument('enable_lidar_avoid', default_value='false')

    # --- 1. Camera chain (USB cam -> jpeg shared mem -> nv12) ---
    usb_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hobot_usb_cam'),
                         'launch', 'hobot_usb_cam.launch.py')),
        launch_arguments={
            'usb_image_width': '1280',
            'usb_image_height': '720',
            'usb_zero_copy': 'True',
            'usb_video_device': LaunchConfiguration('device'),
        }.items())

    nv12_decode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hobot_codec'),
                         'launch', 'hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_channel': '1',
            'codec_in_format': 'jpeg',
            'codec_out_format': 'nv12',
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/nv12_img',
        }.items())

    # --- 2. Odometry fusion stack (provides /odom_combined) ---
    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'akm_fusion.launch.py')),
        launch_arguments={'port': LaunchConfiguration('port')}.items())

    # --- 3. Waypoint navigation (odometry) with vision handoff ---
    waypoint_nav = Node(
        package='origincar_v2_bridge',
        executable='waypoint_nav_node',
        name='waypoint_nav_node',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            waypoints_config,
            {'vision_handoff_wp_idx': LaunchConfiguration('vision_handoff_wp_idx')},
            {'lead_with_vision': LaunchConfiguration('lead_with_vision')},
            {'odom_resume_wp_idx': LaunchConfiguration('odom_resume_wp_idx')},
            {'qr_turnaround_yaw_deg': LaunchConfiguration('qr_turnaround_yaw_deg')},
            {'qr_turnaround_reverse_speed': LaunchConfiguration('qr_turnaround_reverse_speed')},
            {'qr_turnaround_angular': LaunchConfiguration('qr_turnaround_angular')},
            {'qr_turnaround_presteer_sec': LaunchConfiguration('qr_turnaround_presteer_sec')},
            {'route_branch_wp_idx': LaunchConfiguration('route_branch_wp_idx')},
            {'route_merge_wp_idx': LaunchConfiguration('route_merge_wp_idx')},
            {'clockwise_vlm_wp_idx': LaunchConfiguration('clockwise_vlm_wp_idx')},
            {'counterclockwise_vlm_wp_idx': LaunchConfiguration('counterclockwise_vlm_wp_idx')},
            {'vlm_stop_sec': LaunchConfiguration('vlm_stop_sec')},
            {'vlm_capture_delay_sec': LaunchConfiguration('vlm_capture_delay_sec')},
        ],
    )

    # ROI QR reader from the provincial codebase. It subscribes /nv12_img and
    # /racing_obstacle_detection, then publishes /qrcode_result plus legacy
    # compatibility topics /zbar_number and /qrcode_bottom.
    qrcode_roi = Node(
        package='vision_roi_tools',
        executable='roi_qr_reader',
        name='roi_qr_reader',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    image_to_model_roi = Node(
        package='vision_roi_tools',
        executable='person_image_cropper',
        name='person_image_cropper',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    race_status_display = Node(
        package='cockpit_feedback',
        executable='race_status_display',
        name='race_status_display',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    scene_vlm_client = Node(
        package='cockpit_feedback',
        executable='scene_vlm_client',
        name='scene_vlm_client',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # --- 4. Vision perception ---
    track_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racing_track_detection_resnet'),
                         'launch', 'racing_track_detection_resnet.launch.py')))

    obstacle_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racing_obstacle_detection_yolo'),
                         'launch', 'racing_obstacle_detection_yolo.launch.py')))

    # --- 5. Vision control (racing_control -> /racing, control_master -> /cmd_vel) ---
    # control_master only drives /cmd_vel while /vision_enable is true (toggled
    # by waypoint_nav_node for the vision lead and final-leg phases).
    racing_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racing_control'),
                         'launch', 'racing_control.launch.py')),
        launch_arguments={
            'end_y_p': LaunchConfiguration('end_y_p'),
            'p_target_x': LaunchConfiguration('p_target_x'),
            'p_align_tolerance': LaunchConfiguration('p_align_tolerance'),
            'p_approach_x': LaunchConfiguration('p_approach_x'),
            'p_kp': LaunchConfiguration('p_kp'),
            'p_max_angular': LaunchConfiguration('p_max_angular'),
            'p_force_stop_y': LaunchConfiguration('p_force_stop_y'),
            'p_blind_start_y': LaunchConfiguration('p_blind_start_y'),
            'p_blind_x': LaunchConfiguration('p_blind_x'),
            'p_blind_duration': LaunchConfiguration('p_blind_duration'),
            'qr_slow_y_min': LaunchConfiguration('qr_slow_y_min'),
            'qr_slow_y_max': LaunchConfiguration('qr_slow_y_max'),
            'qr_stop_y': LaunchConfiguration('qr_stop_y'),
            'qr_slow_x': LaunchConfiguration('qr_slow_x'),
            'qr_bottom_timeout': LaunchConfiguration('qr_bottom_timeout'),
            'enable_lidar_avoid': LaunchConfiguration('enable_lidar_avoid'),
        }.items())

    return LaunchDescription([
        handoff_arg,
        lead_arg,
        resume_arg,
        route_branch_arg,
        route_merge_arg,
        clockwise_vlm_wp_arg,
        counterclockwise_vlm_wp_arg,
        vlm_stop_arg,
        vlm_capture_delay_arg,
        port_arg,
        device_arg,
        end_y_p_arg,
        p_target_x_arg,
        p_align_tolerance_arg,
        p_approach_x_arg,
        p_kp_arg,
        p_max_angular_arg,
        p_force_stop_y_arg,
        p_blind_start_y_arg,
        p_blind_x_arg,
        p_blind_duration_arg,
        qr_yaw_arg,
        qr_reverse_speed_arg,
        qr_angular_arg,
        qr_presteer_arg,
        qr_slow_y_min_arg,
        qr_slow_y_max_arg,
        qr_stop_y_arg,
        qr_slow_x_arg,
        qr_bottom_timeout_arg,
        enable_lidar_avoid_arg,
        usb_cam,
        nv12_decode,
        fusion,
        waypoint_nav,
        track_detection,
        obstacle_detection,
        qrcode_roi,
        image_to_model_roi,
        race_status_display,
        scene_vlm_client,
        racing_control,
    ])
