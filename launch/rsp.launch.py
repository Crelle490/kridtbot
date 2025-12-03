import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # sim time argument
    use_sim = LaunchConfiguration('use_sim')
    use_skid = LaunchConfiguration('use_skid')
    
    # Process URDF 
    pkg_path = os.path.join(get_package_share_directory('kridtbot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro') # main xacro
    robot_description_config = Command(['xacro ', xacro_file, ' use_sim_system:=', use_sim, ' use_skid:=', use_skid])
    
    # robot_state_publisher node on topic "robot_description"
    params_robot_state_publisher = {'robot_description': robot_description_config, 'use_sim_time': use_sim}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params_robot_state_publisher]
    )

    params_joint_state_publisher = {'robot_description': robot_description_config}
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[params_joint_state_publisher]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )



    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='uses ros2_control if true'),
        DeclareLaunchArgument(
            'use_skid',
            default_value='false',
            description='uses skid if true'),
        node_robot_state_publisher,
        rviz2_node,
    ])
