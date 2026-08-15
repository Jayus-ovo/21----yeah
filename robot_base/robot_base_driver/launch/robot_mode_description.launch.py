import os
from pathlib import Path
import launch_ros.actions
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,LogInfo,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_model_desc = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('robot_description'),'urdf','robot.urdf')]
        )
    ])

    ld = LaunchDescription()

    ld.add_action(robot_model_desc)
    return ld