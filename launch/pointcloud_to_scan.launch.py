from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in','lidar/points/points')],
            parameters=[{
                'target_frame': 'lidar_link',
                'transform_tolerance': 0.01,
                'min_height': -0.2,
                'max_height': 0.2,
                'angle_min': -3.14,  # -M_PI/2
                'angle_max': 3.14,  # M_PI/2
                'angle_increment': 0.017 ,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.8,
                'range_max': 20.0,
                'use_inf': False,
                'inf_epsilon': 1.0,
                'use_time': True,
            }],
            name='pointcloud_to_laserscan'
        )
    ])