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

    # launch lidar, point->scan, slam_toolbox
    slam_bring_up = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("kridtbot"), "launch", "lidar_slam2D.launch.py"),
    )

    # optional: launch 3D lidar slam
    slam_3d_bring_up = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("kridtbot"), "launch", "lidar_slam3D.launch.py"),
    )

    # launch nav2
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
            slam_bring_up,
            #slam_3d_bring_up,
            nav2_bring_up,
        ]
    )