import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Copy config files
    dnn_node_example_path = os.path.join(get_package_prefix('dnn_node_example'), "lib/dnn_node_example")
    os.system(f"cp -r {dnn_node_example_path}/config .")

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument("dnn_example_config_file", default_value=TextSubstitution(text="config/fcosworkconfig.json")),
        DeclareLaunchArgument("dnn_example_dump_render_img", default_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("dnn_example_image_width", default_value=TextSubstitution(text="480")),
        DeclareLaunchArgument("dnn_example_image_height", default_value=TextSubstitution(text="272")),
        DeclareLaunchArgument("dnn_example_msg_pub_topic_name", default_value=TextSubstitution(text="hobot_dnn_detection")),
        DeclareLaunchArgument('device', default_value='/dev/video0', description='usb camera device'),
    ]

    # Include launch descriptions
    usb_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'),
                                       launch_arguments={'usb_image_width': '640', 'usb_image_height': '480','usb_zero_copy': 'True',
                                                         'usb_video_device': LaunchConfiguration('device')}.items())

    nv12_decode_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
                                               launch_arguments={'codec_channel'  : '1',
                                                                 'codec_in_format':'jpeg',        'codec_out_format': 'nv12',
                                                                 'codec_in_mode'  : 'shared_mem', 'codec_out_mode'  : 'shared_mem',
                                                                 'codec_sub_topic': '/hbmem_img', 'codec_pub_topic' : '/nv12_img'}.items())

    img_encode_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'),
                                               launch_arguments={'codec_channel'  : '2',             'codec_jpg_quality': '15.0', 'codec_output_framerate' : '25',
                                                                 'codec_in_format': 'nv12',          'codec_out_format' : 'jpeg',
                                                                 'codec_in_mode'  : 'shared_mem',    'codec_out_mode'   : 'ros',
                                                                 'codec_sub_topic': '/nv12_img', 'codec_pub_topic'  : '/video_img_video'}.items())
                                                                 
    web_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
                                        launch_arguments={'websocket_image_topic': '/image', 'websocket_image_type': 'mjpeg',
                                                          'websocket_smart_topic': LaunchConfiguration("dnn_example_msg_pub_topic_name")}.items())

    racing_obstacle_detection_yolo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_obstacle_detection_yolo') + '/launch/racing_obstacle_detection_yolo.launch.py'))

    racing_track_detection_resnet = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_track_detection_resnet') + '/launch/racing_track_detection_resnet.launch.py'))
    # V2 protocol bridge (replaces old origincar_base)
    # Includes: v2_serial_node + Madgwick + EKF + nonholonomic + ZUPT
    origincar_base = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('origincar_v2_bridge') + '/launch/akm_fusion.launch.py'))
    racing_control = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        get_package_share_directory('racing_control') + '/launch/racing_control.launch.py'))

    # Algorithm node
    dnn_node_example_node = Node(
        package='dnn_node_example',
        executable='example',
        output='screen',
        parameters=[
            {"config_file": LaunchConfiguration('dnn_example_config_file')},
            {"dump_render_img": LaunchConfiguration('dnn_example_dump_render_img')},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {"msg_pub_topic_name": LaunchConfiguration("dnn_example_msg_pub_topic_name")}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    image_transport_node = Node(
        package='utils',
        executable='image_transport_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    vision_language_model_node = Node(
        package='vision_language_model',
        executable='vision_language_model',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    img_to_model = Node(
        package='img_to_model',
        executable='img_to_model',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    qrcode = Node(
        package='qrcode',
        executable='qrcode',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    rosbridge_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )
    return LaunchDescription(launch_args + [
        usb_node,
        nv12_decode_node,
        img_encode_node,
        racing_obstacle_detection_yolo,
        racing_track_detection_resnet,
        vision_language_model_node,
        img_to_model,
        qrcode,
        origincar_base,
        rosbridge_node,
        racing_control,
    ])