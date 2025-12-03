## Launch three packages to run Gazebo
## From template: https://github.com/joshnewans/articubot_one/blob/adb08202d3dafeeaf8a3691ddd64aa8551c40f78/launch/launch_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command

from launch_ros.actions import Node

import subprocess



def generate_launch_description():

    package_name='kridtbot'
    use_mec_or_diff = 'diff' # 'choose between 'mec' or 'diff' for mecanum or differential drive
    
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
    rsp_test = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )])
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
    delayed_controller_manager = TimerAction(period=3.0,
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
        arguments=["linear_velocity_controller","--inactive"],
    )

    delayed_lin_vel_control_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[lin_vel_control_spawner]
        ), condition=UnlessCondition(use_sim_config)
    )


    suspension_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["suspension_controller","--inactive"],
    )

    delayed_suspension_control_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[suspension_control_spawner]
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

        # ros2 jazzy update. no use of unstamped 
    imu_relay_node = Node(
        package='topic_tools',
        executable='throttle',
        parameters=[{'input_topic': '/imu_sensor_broadcaster/imu','msgs_per_sec':20}],
    )

    # compressed imager
    compressed_image = Node(   # see by choose /out/compressed in ros2 run rqt_image_view rqt_image_view
            package="image_transport",
            executable="republish",
            arguments=["raw", "in:=/camera/image_raw", "compressed", "out:=/camera/image_raw/compressed"],
            output="screen",
        )
    
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')

    if use_mec_or_diff == 'mec':

        # Check if IMU topic is available
        timer_for_ukf = TimerAction(
                period=1.0,
                actions=[OpaqueFunction(function=check_node_and_launch_mecanum)]
            )

        mecanum_control_spawner = Node(
            package="controller_manager",
            executable="spawner",
            remappings=[("/mecanum_drive_controller/odometry", "/odom")],
            arguments=["mecanum_drive_controller"],
        )
        # Uncented kalman filter node
        delayed_dirve_system_control_spwaner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[mecanum_control_spawner]
            ), condition=UnlessCondition(use_sim_config)
        )

        twist_mux = Node(
                package="twist_mux",
                executable="twist_mux",
                parameters=[twist_mux_params],
                remappings=[('/cmd_vel_out','/mecanum_drive_controller/cmd_vel_unstamped')]
            )
        twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': False,'frame_id': 'base_link'}],
        remappings=[('/cmd_vel_in', '/mecanum_drive_controller/cmd_vel_unstamped'),
                    ('/cmd_vel_out','/mecanum_drive_controller/reference')]
        )
    elif use_mec_or_diff == 'diff':
        # Differential drive controller for real time system
        diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"],
        )

        delayed_dirve_system_control_spwaner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[diff_drive_spawner]
            ), condition=UnlessCondition(use_sim_config)
        )
        
        timer_for_ukf = TimerAction(
                    period= 1.0,
                    actions=[OpaqueFunction(function=check_node_and_launch_diff)]
                )
        twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
        twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': False,'frame_id': 'base_link'}],
        remappings=[('/cmd_vel_in', '/diff_cont/cmd_vel_unstamped'),
                    ('/cmd_vel_out','/diff_cont/cmd_vel')]
        )
    else:
        raise ValueError("Invalid value for use_mec_or_diff. Expected 'mec' or 'diff'.")



    # Launch all
    return LaunchDescription([
        use_sim,
        #rsp_test,
        #rsp,
        #gz_spawner,
        joystick,
        twist_mux,
        #imu_relay_node,
        #twist_mux_diff_drive,
        twist_stamper,
        #lin_control_spawner_gz,
        #imu_broad_spawner_gz,
        #joint_broad_spawner_gz,
        #diff_drive_spawner_gz,
        delayed_controller_manager,
        delayed_lin_control_joy_spawner,    
        delayed_lin_pos_control_spwaner,    #Only for physical system (calibration)
        delayed_lin_vel_control_spwaner,    #Only for physical system (calibration)
        delayed_joint_broad_spawner,
        #timer_for_ukf,
        delayed_dirve_system_control_spwaner,
        delayed_suspension_control_spwaner,
        #controller_switch_node,
        
        #compressed_image
        #diff_drive_spawner,
        #joint_broad_spawner

    ])

def check_node_and_launch_mecanum(context, *args, **kwargs):
    result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
    if '/mecanum_drive_controller' in result.stdout:
        # Target node is available, launch your dependent node
        return [Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_mecanum",
                output="screen",
                remappings=[
                    ("/odometry/filtered", "/odom")],
                parameters=[os.path.join(get_package_share_directory('kridtbot'), "config", "ekf.yaml")],
            )]
    else:
        # Try again later by scheduling another TimerAction
        return [TimerAction(
            period=1.0,
            actions=[OpaqueFunction(function=check_node_and_launch_mecanum)]
        )]

def check_node_and_launch_diff(context, *args, **kwargs):
    result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
    if '/diff_cont' in result.stdout:
        # Target node is available, launch your dependent node
        return [Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_diff",
                output="screen",
                remappings=[
                    ("/odometry/filtered", "/odom")],
                parameters=[os.path.join(get_package_share_directory('kridtbot'), "config", "ekf.yaml")],
            )]
    else:
        # Try again later by scheduling another TimerAction
        return [TimerAction(
            period=1.0,
            actions=[OpaqueFunction(function=check_node_and_launch_diff)]
        )]