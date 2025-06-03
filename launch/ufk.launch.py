from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    
    ukf_launch = Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_diff",
                output="screen",
                remappings=[
                    ("/odometry/filtered", "/odom")],
                parameters=[os.path.join(get_package_share_directory('kridtbot'), "config", "ekf.yaml")],
            )


    return LaunchDescription(
        [
            ukf_launch,
        ]
    )