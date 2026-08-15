from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions

def generate_launch_description():
    akmcar = LaunchConfiguration('akmcar', default='false')

    robot_parameters = [
        {'usart_port_name': '/dev/ttyACM0',
         'serial_baud_rate': 115200,
         'robot_frame_id': 'base_footprint',
         'odom_frame_id': 'odom_combined',
         'cmd_vel': 'cmd_vel',
         'product_number': 0}
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'akmcar',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='robot_base_driver',
            executable='robot_chassis_node',
            parameters=robot_parameters + [{'akm_cmd_vel': 'ackermann_cmd'}],
            remappings=[('/cmd_vel', 'cmd_vel')],
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='robot_base_driver',
            executable='twist_to_ackermann.py',
            name='twist_to_ackermann',
        ),

        launch_ros.actions.Node(
            condition=UnlessCondition(akmcar),
            package='robot_base_driver',
            executable='robot_chassis_node',
            parameters=robot_parameters + [{'akm_cmd_vel': 'none'}],
        )
    ])