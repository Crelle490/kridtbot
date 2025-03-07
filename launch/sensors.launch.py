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
    # Set ethernet port to register lidar
    set_ip = ExecuteProcess( # set ethernet channel to LIDAR ip
        cmd=['sudo', 'ifconfig', 'eth0', '192.168.1.50'],
        output='screen'
    )

    # launch the ros2 driver from LIVOX
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch', 'msg_MID360_launch.py')
        )
    )

    # Add a static transformation to compensate rotation
    livox_transform = Node( 
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_livox',
        arguments=['0', '0', '0', '0', '3.14', '0', 'livox_mid360_frame', 'livox_frame'],
        output='screen'
    )

    ##### LAUNCH OAK-D
    # Path to the camera launch file
    camera_launch_file = os.path.join(
        get_package_share_directory("depthai_ros_driver"), "launch", "pointcloud.launch.py"
    )

    # Path to YAML config file
    camera_params_file = os.path.join(
        get_package_share_directory("kridtbot"), "config", "camera_params.yaml"
    )

    # Include the camera launch file and pass the YAML parameters
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
        launch_arguments={"params_file": camera_params_file}.items(),
    )

    return LaunchDescription([
        set_ip,
        livox_launch,
        livox_transform,
        camera_launch
    ])
