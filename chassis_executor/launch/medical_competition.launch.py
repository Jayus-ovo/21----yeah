#!/usr/bin/env python3
"""
智慧医疗比赛主启动文件
整合所有功能模块，实现完整的比赛任务流程
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成启动描述"""

    # 启动参数声明
    base_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('robot_base_driver'),
                     'launch', 'base_driver_bringup.launch.py')))
    usb_camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('hobot_usb_cam'),
                     'launch', 'hobot_usb_cam.launch.py')),
        launch_arguments={'usb_image_width': '640', 'usb_image_height': '480',
                          'usb_zero_copy': 'True',
                          'usb_video_device': LaunchConfiguration('device')}.items())
    nv12_decode = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('hobot_codec'),
                     'launch', 'hobot_codec_decode.launch.py')),
        launch_arguments={'codec_channel': '1', 'codec_in_format': 'jpeg',
                          'codec_out_format': 'nv12', 'codec_in_mode': 'shared_mem',
                          'codec_out_mode': 'shared_mem', 'codec_sub_topic': '/hbmem_img',
                          'codec_pub_topic': '/nv12_img'}.items())
    jpeg_encode = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('hobot_codec'),
                     'launch', 'hobot_codec_encode.launch.py')),
        launch_arguments={'codec_channel': '2', 'codec_in_format': 'nv12',
                          'codec_out_format': 'jpeg', 'codec_in_mode': 'shared_mem',
                          'codec_out_mode': 'ros', 'codec_sub_topic': '/nv12_img',
                          'codec_pub_topic': '/jpeg_img'}.items())
    guard_stack = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('perception_yolo'),
                     'launch', 'perception_yolo.launch.py')))
    route_stack = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('perception_track'),
                     'launch', 'track_center_detector.launch.py')))

    launch_args = [
        # 基础参数
        DeclareLaunchArgument('device', default_value='/dev/video0',
                              description='摄像头设备'),
        DeclareLaunchArgument('web_show', default_value='1',
                              description='网页显示开关'),
        DeclareLaunchArgument('save_tuwen_picture', default_value='true',
                              description='保存图文图片'),

        # 导航速度参数
        DeclareLaunchArgument('cruise_speed', default_value='0.7',
                              description='正常循迹速度'),
        DeclareLaunchArgument('steering_gain', default_value='0.0061',
                              description='转向增益'),
        DeclareLaunchArgument('channel_speed', default_value='0.7',
                              description='通道内速度'),

        # 时间限制
        DeclareLaunchArgument('time_limit', default_value='180.0',
                              description='比赛时间限制(秒)'),

        # 语音播报参数
        DeclareLaunchArgument('voice_enabled', default_value='true',
                              description='启用语音播报'),
        DeclareLaunchArgument('screen_enabled', default_value='true',
                              description='启用屏幕显示'),

        # 大模型参数
        DeclareLaunchArgument('api_endpoint',
                              default_value='https://open.volcengineapi.com/ark/v3',
                              description='视觉推理接口地址'),
        DeclareLaunchArgument('api_model',
                              default_value='doubao-lite-128k',
                              description='视觉推理模型标识'),
        DeclareLaunchArgument('vlm_timeout', default_value='30.0',
                              description='大模型超时时间'),
        DeclareLaunchArgument('vlm_max_retries', default_value='1',
                              description='最大重试次数'),

        # 图文检测参数
        DeclareLaunchArgument('margin_tuwen', default_value='0.15',
                              description='图文裁剪边距'),
        DeclareLaunchArgument('capture_confidence', default_value='0.52',
                              description='图文检测置信度'),
        DeclareLaunchArgument('capture_min_area', default_value='4200.0',
                              description='图文检测最小面积'),

        # 定位参数
        DeclareLaunchArgument('position_tolerance', default_value='0.055',
                              description='P点定位容差'),
    ]

    # 底盘执行器节点
    # 任务协调器节点
    mission_coordinator_node = Node(
        package='chassis_executor',
        executable='bravo_route_conductor',
        name='bravo_route_conductor',
        output='screen',
        parameters=[{
            'navigate_speed': 0.7,
            'navigate_angular_gain': 0.0062,
            'position_tolerance': LaunchConfiguration('position_tolerance'),
            'angle_tolerance': 0.09,
            'qr_scan_timeout': 15.0,
            'qr_scan_confidence': 0.72,
            'channel_speed': LaunchConfiguration('channel_speed'),
            'channel_gain': 0.0055,
            'human_confidence': 0.68,
            'human_min_area': 4200.0,
            'max_retry_count': 3,
            'recovery_timeout': 10.0,
            'match_limit_sec': 180.0,
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 播报管理器节点
    track_follower_node = Node(
        package='chassis_executor', executable='track_follower',
        name='bravo_track_follower', output='screen')

    obstacle_avoider_node = Node(
        package='chassis_executor', executable='obstacle_avoider',
        name='bravo_obstacle_avoider', output='screen')

    announcement_manager_node = Node(
        package='chassis_executor',
        executable='announcement_manager',
        name='announcement_manager',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 精准定位控制器
    positioning_controller_node = Node(
        package='chassis_executor',
        executable='positioning_controller',
        name='positioning_controller',
        output='screen',
        parameters=[{
            'position_tolerance': LaunchConfiguration('position_tolerance'),
            'angle_tolerance': 0.09,
            'approach_speed': 0.6,
            'fine_position_speed': 0.6,
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 二维码检测节点
    qr_detector_node = Node(
        package='qr_scanner',
        executable='qr_detector',
        name='qr_detector',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 语音桥接节点
    tts_bridge_node = Node(
        package='voice_alerts',
        executable='tts_bridge',
        name='tts_bridge',
        output='screen',
        parameters=[{
            'qr_result_topic': '/barcode_data',
            'tts_output_topic': '/speech_cmd',
            'announce_once': True,
        }]
    )

    # TTS服务节点
    tts_service_node = Node(
        package='tts_service',
        executable='tts_service_node',
        name='tts_service',
        output='screen',
        parameters=[{
            'topic_sub': '/speech_cmd',
            'playback_device': 'plughw:2,0',
            'volume_gain': 1.0,
            'warmup_enabled': True,
            'disk_cache_enabled': True,
            'common_chars_cache_enabled': False,
        }]
    )

    # 场景分析节点(视觉推理)
    scene_analyzer_node = Node(
        package='scene_analyzer',
        executable='scene_analyzer',
        name='scene_analyzer',
        output='screen',
        parameters=[{
            'save_snapshot': LaunchConfiguration('save_tuwen_picture'),
            'snapshot_dir': '/tmp/scene_snap',
            'crop_margin': LaunchConfiguration('margin_tuwen'),
            'snapshot_quality': 92,
            'api_endpoint': LaunchConfiguration('api_endpoint'),
            'api_model': LaunchConfiguration('api_model'),
            'vlm_timeout': LaunchConfiguration('vlm_timeout'),
            'vlm_max_retries': LaunchConfiguration('vlm_max_retries'),
            'scene_prompt': '识别画面中的医用标识人物模型，用简短语句输出其特征。',
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 车载信息展示面板
    info_display_panel_node = Node(
        package='scene_analyzer',
        executable='info_display_panel',
        name='info_display_panel',
        output='screen',
        additional_env={'DISPLAY': ':0'},
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription(launch_args + [
        base_bringup,
        usb_camera,
        nv12_decode,
        jpeg_encode,
        guard_stack,
        route_stack,
        # 核心控制节点
        # bravo_route_conductor is the only /cmd_vel publisher.

        # 任务管理节点
        mission_coordinator_node,
        track_follower_node,
        obstacle_avoider_node,

        # 播报系统
        announcement_manager_node,

        # 精准定位
        positioning_controller_node,

        # 二维码检测
        qr_detector_node,

        # 语音系统
        tts_bridge_node,
        tts_service_node,

        # 场景分析 + 展示面板
        scene_analyzer_node,
        info_display_panel_node,
    ])
