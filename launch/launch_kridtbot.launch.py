## Launch three packages to run Gazebo
## From template: https://github.com/joshnewans/articubot_one/blob/adb08202d3dafeeaf8a3691ddd64aa8551c40f78/launch/launch_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command

from launch_ros.actions import Node



def generate_launch_description():

    package_name='kridtbot'
    
    use_sim_config = LaunchConfiguration('use_sim')
    
    # Declare use_sim argument
    use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Boolean flag to enable simulation mode'
    )

    #### Launch 1: Launch the robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim':'true'}.items(),
                condition=IfCondition(use_sim_config)
    )

        #### ********** Launch 2: Launch Gazebo or Controller_manager ********** ####
    # Launch gazebo if simulation is enabled
    gz_spawner = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','gz_spawner.launch.py'
                )]), launch_arguments={'use_sim':'true'}.items(),
                condition=IfCondition(use_sim_config)
    )
    
        # Launch controller_manager if simulation is disabled
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
    delayed_controller_manager = TimerAction(period=1.0,
        actions=[controller_manager],
        condition=UnlessCondition(use_sim_config))

    #### *********** Launch 3: configure, inactive and activate controllers ******** ######
    # Differential drive controller for gazebo
    diff_drive_spawner_gz = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        condition=IfCondition(use_sim_config)
    )
    
    # Differential drive controller for real time system
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        ), condition=UnlessCondition(use_sim_config)
    )

    # Joint state broadcaster controller
    joint_broad_spawner_gz = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        condition=IfCondition(use_sim_config)
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
        ), condition=UnlessCondition(use_sim_config)
    )

    # Linear position controller for joystick
    lin_control_spawner_gz = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_position_control_joy"],
        condition=IfCondition(use_sim_config)
    )

    lin_control_joy_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_position_control_joy","--inactive"],
    )

    delayed_lin_control_joy_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[lin_control_joy_spawner]
        ), condition=UnlessCondition(use_sim_config)
    )

    # Linear position controller for commands (used in calibration)
    lin_pos_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_position_control","--inactive"],
    )

    delayed_lin_pos_control_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[lin_pos_control_spawner]
        ), condition=UnlessCondition(use_sim_config)
    )

    # Linear velocity controller for commands (used in calibration)
    lin_vel_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_velocity_controller"],
        
    )

    delayed_lin_vel_control_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[lin_vel_control_spawner]
        ), condition=UnlessCondition(use_sim_config)
    )

    # Automatic suspension control
    suspension_control_spawner_gz = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["suspension_controller"],
        condition=IfCondition(use_sim_config)
    )

    suspension_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["suspension_controller","--active"],
    )

    delayed_suspension_control_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[suspension_control_spawner]
        ), condition=UnlessCondition(use_sim_config)
    )

    # IMU controller
    imu_broad_spawner_gz = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["imu_sensor_broadcaster"],
        output='screen',
        condition=IfCondition(use_sim_config)
    )

    imu_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["imu_sensor_broadcaster"],
        output='screen'
    ) 
    delayed_imu_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[imu_broad_spawner]
        ), condition=UnlessCondition(use_sim_config)
    )


 #### ********* Launch 4: Miscanellous ********* ####
    # Joystick
    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','joystick.launch.py'
            )])
    )

    # ros2 jazzy update. no use of unstamped 
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': False}],
        remappings=[('/cmd_vel_in', 'diff_cont/cmd_vel_unstamped'),
                    ('/cmd_vel_out','/diff_cont/cmd_vel')]
    )

    # compressed imager
    compressed_image = Node(   # see by choose /out/compressed in ros2 run rqt_image_view rqt_image_view
            package="image_transport",
            executable="repsuspension_controllerublish",
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
        use_sim,
        rsp,
        gz_spawner,
        joystick,
        twist_stamper,
        #lin_control_spawner_gz,
        imu_broad_spawner_gz,
        joint_broad_spawner_gz,
        diff_drive_spawner_gz,
        #suspension_control_spawner_gz,
        delayed_controller_manager,
        delayed_imu_broad_spawner,
        delayed_lin_control_joy_spawner,    
        delayed_lin_pos_control_spwaner,    #Only for physical system (calibration)
        delayed_lin_vel_control_spwaner,    #Only for physical system (calibration)
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
        #delayed_suspension_control_spwaner,
        port_arg,
        foxglove_bridge
        #compressed_image
        #diff_drive_spawner,
        #joint_broad_spawner
    ])