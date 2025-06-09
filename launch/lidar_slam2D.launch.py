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

    set_ip = ExecuteProcess( # set ethernet channel to LIDAR ip
        cmd=['sudo', 'ifconfig', 'enp1s0', '192.168.1.50'],
        output='screen'
    )
    
    lidar_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("livox_ros_driver2"), "launch", "msg_MID360_launch.py"),
    )

    point_to_scan = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("kridtbot"), "launch", "pointcloud_to_scan.launch.py"),
    )

    slam_toolbox = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"),
        launch_arguments={
            "params_file": PathJoinSubstitution(
                [get_package_share_directory("kridtbot"), "config", "mapper_params.yaml"]
            ),
        }.items()
    )


    return LaunchDescription(
        [
            #set_ip,
            #lidar_launch,
            point_to_scan,
            slam_toolbox,
        ]
    )

