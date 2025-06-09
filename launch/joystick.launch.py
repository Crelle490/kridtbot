from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('kridtbot'), 'config', 'joy_params.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[("cmd_vel", "/diff_cont/cmd_vel_unstamped")]
    )

    linear_pos = TimerAction(
        period=2.0, 
        actions=[Node(
            package='kridtbot',
            executable='linear_position_pub',
            name='linear_position_pub',
            parameters=[joy_params],
        )]
    )


    return LaunchDescription([
        joy_node,
        teleop_node,
        linear_pos,
    ])