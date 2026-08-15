from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cruise_speed', default_value='0.7'),
        DeclareLaunchArgument('steering_gain', default_value='0.0055'),
        DeclareLaunchArgument('yellow_speed', default_value='0.7'),
        DeclareLaunchArgument('yellow_gain', default_value='0.0056'),
        DeclareLaunchArgument('center_go', default_value='315.0'),
        DeclareLaunchArgument('center_back', default_value='315.0'),
        DeclareLaunchArgument('center_yellow', default_value='322.0'),
        DeclareLaunchArgument('stop_line_y', default_value='255'),
        DeclareLaunchArgument('capture_wait', default_value='6.5'),
        DeclareLaunchArgument('min_confidence', default_value='0.52'),
        DeclareLaunchArgument('min_frames', default_value='2'),

        DeclareLaunchArgument('resnet_timeout', default_value='0.18'),
        DeclareLaunchArgument('yolo_timeout', default_value='0.18'),
        DeclareLaunchArgument('drift_stop_frames', default_value='3'),
        DeclareLaunchArgument('drift_duration', default_value='40'),
        DeclareLaunchArgument('drift_velocity', default_value='-0.7'),
        DeclareLaunchArgument('drift_angular', default_value='4.5'),
        DeclareLaunchArgument('post_drift_speed', default_value='0.6'),
        DeclareLaunchArgument('post_drift_angular', default_value='0.0'),
        DeclareLaunchArgument('post_drift_stop_frames', default_value='3'),
        DeclareLaunchArgument('resnet_ready_threshold', default_value='3'),
        DeclareLaunchArgument('yellow_exit_threshold', default_value='155'),
        DeclareLaunchArgument('yellow_blend_in', default_value='0.25'),
        DeclareLaunchArgument('post_drift_timeout', default_value='4.2'),

        DeclareLaunchArgument('avoid_speed', default_value='0.7'),
        DeclareLaunchArgument('avoid_gain', default_value='0.0035'),
        DeclareLaunchArgument('avoid_center_x', default_value='335.0'),
        DeclareLaunchArgument('force_right_avoid', default_value='0'),
        DeclareLaunchArgument('point_speed', default_value='0.6'),
        DeclareLaunchArgument('point_gain', default_value='0.0035'),
        DeclareLaunchArgument('point_center_x', default_value='322.0'),
        DeclareLaunchArgument('qr_speed', default_value='0.6'),
        DeclareLaunchArgument('qr_gain', default_value='0.0035'),
        DeclareLaunchArgument('qr_center_x', default_value='322.0'),
        DeclareLaunchArgument('enable_qr_tracking', default_value='1'),
        DeclareLaunchArgument('conf_threshold_p', default_value='0.72'),
        DeclareLaunchArgument('conf_threshold_qr', default_value='0.72'),
        DeclareLaunchArgument('conf_threshold_zt', default_value='0.72'),
        DeclareLaunchArgument('min_frames_p', default_value='3'),
        DeclareLaunchArgument('min_frames_qr', default_value='3'),
        DeclareLaunchArgument('min_frames_avoid', default_value='3'),
        DeclareLaunchArgument('lost_hold_frames_p', default_value='15'),
        DeclareLaunchArgument('lost_hold_frames_qr', default_value='15'),
        DeclareLaunchArgument('lost_slow_frames_p', default_value='45'),
        DeclareLaunchArgument('lost_slow_frames_qr', default_value='45'),
        DeclareLaunchArgument('lost_hold_speed_p', default_value='0.7'),
        DeclareLaunchArgument('lost_hold_speed_qr', default_value='0.7'),
        DeclareLaunchArgument('reverse_speed_p', default_value='0.7'),
        DeclareLaunchArgument('reverse_speed_qr', default_value='0.7'),
        DeclareLaunchArgument('reverse_slow_speed_p', default_value='0.6'),
        DeclareLaunchArgument('reverse_slow_speed_qr', default_value='0.6'),

        DeclareLaunchArgument('capture_speed', default_value='0.0'),
        DeclareLaunchArgument('capture_gain', default_value='0.0055'),
        DeclareLaunchArgument('capture_delay', default_value='8.5'),
        DeclareLaunchArgument('capture_confidence', default_value='0.52'),
        DeclareLaunchArgument('capture_min_area', default_value='4200.0'),
        DeclareLaunchArgument('capture_min_y', default_value='155.0'),
        DeclareLaunchArgument('capture_frame_count', default_value='3'),
        DeclareLaunchArgument('capture_blend_in', default_value='0.0'),
        DeclareLaunchArgument('capture_blend_out', default_value='0.25'),
        DeclareLaunchArgument('capture_settle_time', default_value='0.25'),

        DeclareLaunchArgument('stop_line_zt', default_value='148'),
        DeclareLaunchArgument('stop_line_p', default_value='425'),
        DeclareLaunchArgument('stop_line_qr', default_value='160'),

        Node(
            package='chassis_executor',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            output='screen',
            parameters=[{
                'avoid_speed': LaunchConfiguration('avoid_speed'),
                'avoid_gain': LaunchConfiguration('avoid_gain'),
                'avoid_center_x': ParameterValue(
                    LaunchConfiguration('avoid_center_x'), value_type=float),
                'force_right_avoid': ParameterValue(
                    LaunchConfiguration('force_right_avoid'),
                    value_type=int),
                'point_speed': ParameterValue(
                    LaunchConfiguration('point_speed'), value_type=float),
                'point_gain': ParameterValue(
                    LaunchConfiguration('point_gain'), value_type=float),
                'point_center_x': ParameterValue(
                    LaunchConfiguration('point_center_x'), value_type=float),
                'qr_speed': ParameterValue(
                    LaunchConfiguration('qr_speed'), value_type=float),
                'qr_gain': ParameterValue(
                    LaunchConfiguration('qr_gain'), value_type=float),
                'qr_center_x': ParameterValue(
                    LaunchConfiguration('qr_center_x'), value_type=float),
                'enable_qr_tracking': ParameterValue(
                    LaunchConfiguration('enable_qr_tracking'),
                    value_type=int),
                'conf_threshold_p': ParameterValue(
                    LaunchConfiguration('conf_threshold_p'), value_type=float),
                'conf_threshold_qr': ParameterValue(
                    LaunchConfiguration('conf_threshold_qr'), value_type=float),
                'conf_threshold_zt': ParameterValue(
                    LaunchConfiguration('conf_threshold_zt'), value_type=float),
                'min_frames_p': ParameterValue(
                    LaunchConfiguration('min_frames_p'), value_type=int),
                'min_frames_qr': ParameterValue(
                    LaunchConfiguration('min_frames_qr'), value_type=int),
                'min_frames_avoid': ParameterValue(
                    LaunchConfiguration('min_frames_avoid'), value_type=int),
                'lost_hold_frames_p': ParameterValue(
                    LaunchConfiguration('lost_hold_frames_p'), value_type=int),
                'lost_hold_frames_qr': ParameterValue(
                    LaunchConfiguration('lost_hold_frames_qr'), value_type=int),
                'lost_slow_frames_p': ParameterValue(
                    LaunchConfiguration('lost_slow_frames_p'), value_type=int),
                'lost_slow_frames_qr': ParameterValue(
                    LaunchConfiguration('lost_slow_frames_qr'),
                    value_type=int),
                'lost_hold_speed_p': ParameterValue(
                    LaunchConfiguration('lost_hold_speed_p'), value_type=float),
                'lost_hold_speed_qr': ParameterValue(
                    LaunchConfiguration('lost_hold_speed_qr'), value_type=float),
                'reverse_speed_p': ParameterValue(
                    LaunchConfiguration('reverse_speed_p'), value_type=float),
                'reverse_speed_qr': ParameterValue(
                    LaunchConfiguration('reverse_speed_qr'), value_type=float),
                'reverse_slow_speed_p': ParameterValue(
                    LaunchConfiguration('reverse_slow_speed_p'), value_type=float),
                'reverse_slow_speed_qr': ParameterValue(
                    LaunchConfiguration('reverse_slow_speed_qr'),
                    value_type=float),

                'stop_line_p': LaunchConfiguration('stop_line_p'),
                'stop_line_zt': LaunchConfiguration('stop_line_zt'),
                'stop_line_qr': LaunchConfiguration('stop_line_qr'),
            }]
        ),

        Node(
            package='chassis_executor',
            executable='track_follower',
            name='track_follower',
            output='screen',
            parameters=[{
                'steering_gain': LaunchConfiguration('steering_gain'),
                'cruise_speed': LaunchConfiguration('cruise_speed'),
                'yellow_gain': LaunchConfiguration('yellow_gain'),
                'yellow_speed': LaunchConfiguration('yellow_speed'),
                'center_go': ParameterValue(
                    LaunchConfiguration('center_go'), value_type=float),
                'center_back': ParameterValue(
                    LaunchConfiguration('center_back'), value_type=float),
                'center_yellow': ParameterValue(
                    LaunchConfiguration('center_yellow'), value_type=float),
                'stop_line_y': LaunchConfiguration('stop_line_y'),
                'capture_wait': ParameterValue(
                    LaunchConfiguration('capture_wait'), value_type=float),
                'min_confidence': ParameterValue(
                    LaunchConfiguration('min_confidence'), value_type=float),
                'min_frames': ParameterValue(
                    LaunchConfiguration('min_frames'), value_type=int),
            }]
        ),

        Node(
            package='chassis_executor',
            executable='chassis_executor_node',
            name='chassis_executor_node',
            output='screen',
            parameters=[{
                'cruise_speed': ParameterValue(
                    LaunchConfiguration('cruise_speed'), value_type=float),
                'steering_gain': ParameterValue(
                    LaunchConfiguration('steering_gain'), value_type=float),
                'yellow_speed': ParameterValue(
                    LaunchConfiguration('yellow_speed'), value_type=float),
                'yellow_gain': ParameterValue(
                    LaunchConfiguration('yellow_gain'), value_type=float),
                'resnet_timeout': ParameterValue(
                    LaunchConfiguration('resnet_timeout'), value_type=float),
                'yolo_timeout': ParameterValue(
                    LaunchConfiguration('yolo_timeout'), value_type=float),
                'drift_stop_frames': ParameterValue(
                    LaunchConfiguration('drift_stop_frames'), value_type=int),
                'drift_duration': ParameterValue(
                    LaunchConfiguration('drift_duration'), value_type=int),
                'drift_velocity': ParameterValue(
                    LaunchConfiguration('drift_velocity'), value_type=float),
                'drift_angular': ParameterValue(
                    LaunchConfiguration('drift_angular'), value_type=float),
                'post_drift_speed': ParameterValue(
                    LaunchConfiguration('post_drift_speed'), value_type=float),
                'post_drift_angular': ParameterValue(
                    LaunchConfiguration('post_drift_angular'),
                    value_type=float),
                'post_drift_stop_frames': ParameterValue(
                    LaunchConfiguration('post_drift_stop_frames'),
                    value_type=int),
                'resnet_ready_threshold': ParameterValue(
                    LaunchConfiguration('resnet_ready_threshold'),
                    value_type=int),
                'yellow_exit_threshold': ParameterValue(
                    LaunchConfiguration('yellow_exit_threshold'),
                    value_type=int),
                'yellow_blend_in': ParameterValue(
                    LaunchConfiguration('yellow_blend_in'),
                    value_type=float),
                'post_drift_timeout': ParameterValue(
                    LaunchConfiguration('post_drift_timeout'),
                    value_type=float),
                'capture_speed': ParameterValue(
                    LaunchConfiguration('capture_speed'), value_type=float),
                'capture_gain': ParameterValue(
                    LaunchConfiguration('capture_gain'), value_type=float),
                'capture_delay': ParameterValue(
                    LaunchConfiguration('capture_delay'), value_type=float),
                'capture_confidence': ParameterValue(
                    LaunchConfiguration('capture_confidence'),
                    value_type=float),
                'capture_min_area': ParameterValue(
                    LaunchConfiguration('capture_min_area'), value_type=float),
                'capture_min_y': ParameterValue(
                    LaunchConfiguration('capture_min_y'), value_type=float),
                'capture_frame_count': ParameterValue(
                    LaunchConfiguration('capture_frame_count'), value_type=int),
                'capture_blend_in': ParameterValue(
                    LaunchConfiguration('capture_blend_in'), value_type=float),
                'capture_blend_out': ParameterValue(
                    LaunchConfiguration('capture_blend_out'), value_type=float),
                'capture_settle_time': ParameterValue(
                    LaunchConfiguration('capture_settle_time'),
                    value_type=float),
            }],
        ),
    ])
