from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('kridtbot'), 'config', 'joy_params.yaml')
    smoother_params = os.path.join(get_package_share_directory('kridtbot'),'config','velocity_smoother_params.yaml')

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
        remappings=[("cmd_vel", "cmd_vel_joy")],
    )

    teleop_node_for_smooth = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[("cmd_vel", "cmd_vel_joy_unsmoothed")],
    )

    vel_smoother = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='nav2_velocity_smoother_for_diff_drive',
        namespace = '',
        parameters=[smoother_params],
        remappings=[("cmd_vel", "cmd_vel_joy_unsmoothed"),
                    ("cmd_vel_smoothed", "cmd_vel_joy")],
    )

    # Configure the lifecycle node
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == vel_smoother,
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )
    activate = TimerAction(
        period=6.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda node: node == vel_smoother,
                    transition_id=Transition.TRANSITION_ACTIVATE
                )
            )
        ]
    )

    # Activate after configure
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=vel_smoother,
            start_state='inactive',
            goal_state='active',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == vel_smoother,
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    linear_pos = TimerAction(
        period=2.0,  # Wait 2 seconds before starting
        actions=[Node(
            package='kridtbot',
            executable='linear_position_pub',
            name='linear_position_pub',
            parameters=[joy_params],
        )]
    )

    switch_node = Node(
        package='kridtbot',
        executable='switch_controller',
        name='switch_node',
        parameters=[joy_params],
    )


    return LaunchDescription([
        joy_node,
        teleop_node,
        linear_pos,
        switch_node
        #vel_smoother,
        #configure_event,
        #activate_event,
        #activate,
        #linear_pos,
    ])