## Launch three packages to run Gazebo
## From template: https://github.com/joshnewans/articubot_one/blob/adb08202d3dafeeaf8a3691ddd64aa8551c40f78/launch/launch_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.actions import Node



def generate_launch_description():
        # Foxglove WebSocket port
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Port number for Foxglove Bridge'
    )
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{"port": LaunchConfiguration('port')}],
        output='screen'
    )

    
    # Launch all
    return LaunchDescription([
        port_arg,
        foxglove_bridge,
    ])