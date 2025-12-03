## Launch three packages to run Gazebo
## From template: https://github.com/joshnewans/articubot_one/blob/adb08202d3dafeeaf8a3691ddd64aa8551c40f78/launch/launch_sim.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    package_name='kridtbot' 

    # Find world file
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'mars.world'
        )    

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    
    # Gazebo launch file
    params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 --render-engine ogre2 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Spawner node from ros_gz_sim package. 
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                        '-name', 'my_bot',
                        '-x', '0', '-y', '5', '-z', '2.43', #-x', '0', '-y', '5', '-z', '2.45',
                        '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
                        output='screen')
    
    # GZ to ros2 topic bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
   
    
    # Launch all
    return LaunchDescription([
 
        world_arg,
        gazebo,
        spawn_entity,
        ros_gz_bridge,

    ])