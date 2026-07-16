import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('line_x',default_value='0.7'),
        DeclareLaunchArgument('line_kp',default_value='0.006'),
        DeclareLaunchArgument('line_center_offset',default_value='20.0'),  # 巡线目标偏移：正值线在图像右侧(车偏左)，负值线在左侧(车偏右)，0居中
        
        
        DeclareLaunchArgument('avoid_x',default_value='0.35'),
        DeclareLaunchArgument('avoid_kp',default_value='0.0050'),
        DeclareLaunchArgument('end_y',default_value='212'),      # 障碍物规避最小bottom阈值（比200稍晚触发）
        
        DeclareLaunchArgument('end_y_p',default_value='435'),      # P点停车触发阈值（越大越靠近P点后停车）
        DeclareLaunchArgument('p_target_x',default_value='290.0'),  # P中心在画面的目标x：减小=车往右靠，增大=车往左靠
        DeclareLaunchArgument('p_align_tolerance',default_value='40.0'),
        DeclareLaunchArgument('p_approach_x',default_value='0.40'),
        DeclareLaunchArgument('p_kp',default_value='0.006'),
        DeclareLaunchArgument('p_max_angular',default_value='1.2'),
        DeclareLaunchArgument('p_force_stop_y',default_value='470'),
        DeclareLaunchArgument('p_blind_start_y',default_value='330'),
        DeclareLaunchArgument('p_blind_x',default_value='0.30'),
        DeclareLaunchArgument('p_blind_duration',default_value='0.8'),

        DeclareLaunchArgument('qr_slow_y_min',default_value='10'),  # 二维码进入画面更早开始减速
        DeclareLaunchArgument('qr_slow_y_max',default_value='80'),  # bottom≥80 降到最低速
        DeclareLaunchArgument('qr_stop_y',default_value='130'),
        DeclareLaunchArgument('qr_slow_x',default_value='0.10'),      # 最低前进速度（更低，防惯性冲撞）
        DeclareLaunchArgument('qr_bottom_timeout',default_value='0.8'),

        DeclareLaunchArgument('enable_lidar_avoid',default_value='false'),
        DeclareLaunchArgument('lidar_avoid_distance',default_value='0.35'),      # 阶段2雷达避障触发距离
        DeclareLaunchArgument('lidar_avoid_clear_distance',default_value='0.55'), # 阶段2雷达避障释放距离
        DeclareLaunchArgument('lidar_bypass_release_margin',default_value='0.15'), # 阶段2绕过障碍物后回归边距
        
        Node(
            package='racing_control',
            executable='racing_control',
            output='screen',
            parameters=[
                {"pub_control_topic": '/racing'},
                {"end_y": LaunchConfiguration('end_y')},
                {"line_x": LaunchConfiguration('line_x')},
                {"line_kp": LaunchConfiguration('line_kp')},
                {"line_center_offset": LaunchConfiguration('line_center_offset')},
                {"avoid_x": LaunchConfiguration('avoid_x')},
                {"avoid_kp": LaunchConfiguration('avoid_kp')},
                {"end_y_p": LaunchConfiguration('end_y_p')},
                {"p_target_x": LaunchConfiguration('p_target_x')},
                {"p_align_tolerance": LaunchConfiguration('p_align_tolerance')},
                {"p_approach_x": LaunchConfiguration('p_approach_x')},
                {"p_kp": LaunchConfiguration('p_kp')},
                {"p_max_angular": LaunchConfiguration('p_max_angular')},
                {"p_force_stop_y": LaunchConfiguration('p_force_stop_y')},
                {"p_blind_start_y": LaunchConfiguration('p_blind_start_y')},
                {"p_blind_x": LaunchConfiguration('p_blind_x')},
                {"p_blind_duration": LaunchConfiguration('p_blind_duration')},
                {"qr_slow_y_min": LaunchConfiguration('qr_slow_y_min')},
                {"qr_slow_y_max": LaunchConfiguration('qr_slow_y_max')},
                {"qr_stop_y": LaunchConfiguration('qr_stop_y')},
                {"qr_slow_x": LaunchConfiguration('qr_slow_x')},
                {"qr_bottom_timeout": LaunchConfiguration('qr_bottom_timeout')},
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='racing_control',
            executable='control_master',
            output='screen',
            parameters=[
                {"enable_lidar_avoid": LaunchConfiguration('enable_lidar_avoid')},
                {"lidar_avoid_distance": LaunchConfiguration('lidar_avoid_distance')},
                {"lidar_avoid_clear_distance": LaunchConfiguration('lidar_avoid_clear_distance')},
                {"lidar_bypass_release_margin": LaunchConfiguration('lidar_bypass_release_margin')},
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])
