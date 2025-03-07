## Launch three packages to run Gazebo
## From template: https://github.com/joshnewans/articubot_one/blob/adb08202d3dafeeaf8a3691ddd64aa8551c40f78/launch/launch_sim.launch.py

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

    package_name='kridtbot' 


    robot_description = Command([
        'ros2 param get --hide-type /robot_state_publisher robot_description'
    ])
    controller_params = os.path.join(get_package_share_directory(package_name), 'config', 'controller_manager.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description':robot_description},
                    controller_params],
    )
    delayed_controller_manager = TimerAction(period=1.0,actions=[controller_manager])
    
    # configure, inactive and activate controllers - diff_drive and joint broadcaster
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','joystick.launch.py'
            )])
    )

    # ros2 jazzy update. no use of unstamped 
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[('/cmd_vel_in', 'diff_cont/cmd_vel_unstamped'),
                    ('/cmd_vel_out','/diff_cont/cmd_vel')]
    )

    lin_pos_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_position_control","--inactive"],
    )

    delayed_lin_pos_control_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[lin_pos_control_spawner]
        )
    )
    lin_vel_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_velocity_controller","--inactive"],
    )

    delayed_lin_vel_control_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[lin_vel_control_spawner]
        )
    )
    
    lin_pos_control_spawner_for_joy = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_position_control_joy","--inactive"],
    )

    delayed_lin_pos_control_for_joy_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[lin_pos_control_spawner_for_joy]
        )
    )

    imu_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"],
    )

    delayed_imu_control_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[imu_control_spawner]
        )
    )

    ## Launch Sensors
    # Launch sensors.launch.py

    # Launch all
    return LaunchDescription([
        delayed_controller_manager,
        twist_stamper,
        joystick,
        delayed_lin_pos_control_for_joy_spwaner,
        delayed_lin_pos_control_spwaner,
        delayed_lin_vel_control_spwaner,
        delayed_diff_drive_spwaner,
        delayed_joint_broad_spawner,
        delayed_imu_control_spwaner,
    ])