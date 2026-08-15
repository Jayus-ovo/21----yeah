import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.path.join(
        get_package_prefix('perception_track'), 'lib', 'perception_track',
        'config', 'bravo_centerline.bin')

    track_detector_node = Node(
        package='perception_track',
        executable='track_center_detector',
        name='track_center_detector',
        output='screen',
        parameters=[
            {"sub_img_topic": "/nv12_img"},
            {"model_path": model_path},
            {"mode_name": "yellow"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([track_detector_node])
