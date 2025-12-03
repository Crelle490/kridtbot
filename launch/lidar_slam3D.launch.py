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
    pkg_dir = get_package_share_directory("kridtbot")
    
    main_param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(pkg_dir, "config", "lidar_slam.yaml"),
    )

    mapping = Node(
        package="scanmatcher",
        executable="scanmatcher_node",
        parameters=[main_param_dir, {"use_sim_time": False}],
        remappings=[
            ("/input_cloud", "/livox/points"),
        ],
        output="screen",
    )

    graph_based_slam = Node(
        package="graph_based_slam",
        executable="graph_based_slam_node",
        parameters=[main_param_dir, {"use_sim_time": False}],
        output="screen",
    )

    return LaunchDescription(
        [           
            DeclareLaunchArgument(
                "param_dir",
                default_value=main_param_dir,
                description="Full path to main parameter file to load",
            ),
            DeclareLaunchArgument(
                "use_sim",
                default_value="False",
                description="Use /clock as time source.",
            ),
            mapping,
            graph_based_slam,
        ]
    )