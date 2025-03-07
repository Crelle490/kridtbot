## Launch three packages to run Gazebo
## From template: https://github.com/joshnewans/articubot_one/blob/adb08202d3dafeeaf8a3691ddd64aa8551c40f78/launch/launch_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    package_name='kridtbot' 

    #### Launch 1: Launch the robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_sim.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control':'true'}.items()
    )

    #### Launch 2: Launch Gazebo and run
    # Find world file
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    
    # Include the Gazebo launch file, provided by the ros_gz_sim package
    params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 --render-engine ogre2 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.3'],
                        output='screen')
    
    # GZ and ros2 topic bridge
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
    #### Launch 3: configure, inactive and activate controllers
    # Differential drive controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # Joint state broadcaster controller
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Linear position controller
    lin_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_position_control"],
    )

    # IMU controller
    imu_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["imu_broad"],
        output='screen'
    )

    #### Launch 4: Miscanellous
    # Joystick
    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','joystick.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ros2 jazzy update. no use of unstamped 
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in', 'diff_cont/cmd_vel_unstamped'),
                    ('/cmd_vel_out','/diff_cont/cmd_vel')]
    )

    # compressed imager
    compressed_image = Node(   # see by choose /out/compressed in ros2 run rqt_image_view rqt_image_view
            package="image_transport",
            executable="republish",
            arguments=["raw", "in:=/camera/image_raw", "compressed", "out:=/camera/image_raw/compressed"],
            output="screen",
        )
    
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
        rsp,
        world_arg,
        gazebo,
        spawn_entity,
        joystick,
        twist_stamper,
        diff_drive_spawner,
        joint_broad_spawner,
        imu_broad_spawner,
        ros_gz_bridge,
        lin_control_spawner,
        #compressed_image,
        #foxglove_bridge
    ])