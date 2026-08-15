import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file = os.path.join(
        get_package_prefix('perception_yolo'), 'lib', 'perception_yolo',
        'config', 'yolov5sconfig_0721.json')
    image_width_arg = DeclareLaunchArgument(
        "dnn_image_width", default_value=TextSubstitution(text="640")
    )
    image_height_arg = DeclareLaunchArgument(
        "dnn_image_height", default_value=TextSubstitution(text="480")
    )
    node_name_arg = DeclareLaunchArgument(
        "node_name", default_value=TextSubstitution(text="perception_yolo")
    )
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=TextSubstitution(
            text=config_file
        ),
    )
    pub_topic_arg = DeclareLaunchArgument(
        "pub_ai_topic", default_value=TextSubstitution(text="/perception_yolo_detection")
    )

    obstacle_detector_node = Node(
        package='perception_yolo',
        executable='perception_yolo',
        name=LaunchConfiguration("node_name"),
        output='screen',
        parameters=[
            {"is_shared_mem_sub": True},
            {"sub_img_topic": "/nv12_img"},
            {"pub_ai_topic": LaunchConfiguration("pub_ai_topic")},
            {"config_file": LaunchConfiguration("config_file")},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        image_width_arg,
        image_height_arg,
        node_name_arg,
        config_file_arg,
        pub_topic_arg,
        obstacle_detector_node
    ])
