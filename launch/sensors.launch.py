import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.actions import RegisterEventHandler, ExecuteProcess 
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():
    ##### LAUNCH LIDAR

    ##### LAUNCH OAK-D
    # Path to the camera launch file
    camera_launch_file = os.path.join(
        get_package_share_directory("depthai_ros_driver"), "launch", "pointcloud.launch.py"
    )

    # Path to YAML config file
    #camera_params_file = os.path.join(
    #    get_package_share_directory("kridtbot"), "config", "camera_params.yaml"
    #)

    # Include the camera launch file and pass the YAML parameters
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file)
    )

    return LaunchDescription([
        camera_launch
    ])
