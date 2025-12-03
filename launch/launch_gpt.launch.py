import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Define your Python virtual environment path
    python_env_activate = os.path.expanduser('~/vosk_venv/bin/activate')

    return LaunchDescription([
        # Activate Python environment and launch Vosk STT
        ExecuteProcess(
            cmd=['bash', '-c', f'source {python_env_activate} && ros2 launch voskros voskros.launch.yaml'],
            output='screen'
        ),

        # GPT ROS2 C++ Node
        Node(
            package='kridtbot',
            executable='gpt_pose_extractor',
            output='screen'
        ),
    ])
