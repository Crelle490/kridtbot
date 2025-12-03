from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    kridtbot_path = get_package_share_directory("kridtbot")
    
    robot_localization_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node_diff",
        output="screen",
        remappings=[
                    ("/odometry/filtered", "/odom")],
        parameters=[os.path.join(kridtbot_path, "config", "ukf.yaml"), {"use_sim_time": False}],
    )


    return LaunchDescription(
        [
            robot_localization_node,
        ]
    )