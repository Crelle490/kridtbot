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

    nav2_bring_up = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": PathJoinSubstitution(
                [get_package_share_directory("kridtbot"), "config", "nav2_params.yaml"]
            ),
        }.items()
    )


    return LaunchDescription(
        [
            #set_ip,
            #lidar_launch,
            #point_to_scan,
            #slam_toolbox,
            nav2_bring_up,
        ]
    )