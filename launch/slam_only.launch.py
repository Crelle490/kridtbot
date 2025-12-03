from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess 
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    kridtbot_path = get_package_share_directory("kridtbot")

    slam_toolbox = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"),
        launch_arguments={
            "params_file": PathJoinSubstitution(
                [get_package_share_directory("kridtbot"), "config", "mapper_params_online_async.yaml"]
            ),
        }.items()
    )



    return LaunchDescription(
        [
            slam_toolbox,
            #nav2_bring_up,
        ]
    )