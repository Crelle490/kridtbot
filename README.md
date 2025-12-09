# KRIDTBOT

![Kridtbot field test with mecanum wheels](https://github.com/Crelle490/kridtbot/blob/main/media/kridtbot.png)

kridtbot is a ROS 2 (ament_cmake) package that bundles the launch files, configurations, and custom nodes used to simulate and drive the Kridtbot mobile platform. It provides Gazebo integration, ros2_control wiring, joystick helpers, and example autonomy utilities so the same package can be used in simulation or on hardware.

See [AUTHORS.md](AUTHORS.md) for full credits.


## Package layout
- **nodes/**: C++ executables for teleoperation, controller switching, and demo behaviors.
- **launch/**: Launch descriptions for simulation, navigation, localization, sensor bridges, and auxiliary tools.
- **config/**: Controller, joystick, navigation, filter and SLAM configuration YAML files referenced by the launch descriptions.
- **description/** and **meshes/**: URDF/xacro description and 3D assets for the robot model.
- **worlds/**: Gazebo world definitions for simulation scenes.

## Notable nodes
- **linear_position_pub** (`nodes/linear_position_pub.cpp`): Maps a joystick axis into linear position commands and publishes a four-element `Float64MultiArray` to `/linear_position_control_joy/commands` for suspension or actuator control.
- **square_driver** (`nodes/square_driver.cpp`): Publishes `/mecanum_drive_controller/reference` velocity commands that drive the robot in a square path, alternating forward motion and 90° turns.
- **FKtest** (`nodes/FKtest.cpp`): Emits wheel velocity commands on `/one_wheel_trajectory/commands` to trace a square pattern using open-loop motor speeds.
- **imu_covariance_node** (`nodes/imu_covariance.cpp`): Copies IMU messages from `imu_sensor_broadcaster/imu` and zeroes the covariance matrices before republishing on `/imu_cov`.
- **controller_switcher** (`nodes/SwitchController.cpp`): Listens to joystick button 5 to toggle between `suspension_controller` and `linear_position_control_joy` via the controller manager switch service.
- **chatgpt_velocity_commander** (`nodes/gptPoseExtractor.cpp`): Subscribes to `/stt/result`, calls the OpenAI Chat Completions API (requires `OPENAI_API_KEY`), parses a JSON velocity command, and republishes it to `/diff_cont/cmd_vel_unstamped` at 10 Hz.
- **switch_suspension_control_mode** (`nodes/switch_suspension_control_mode.cpp`): Skeleton node prepared for switching suspension modes from a joystick axis.

## Key launch files
- **launch_kridtbot.launch.py**: Orchestrates robot_state_publisher, Gazebo, and ros2_control controller spawners. Accepts `use_sim` to toggle between simulation and hardware flows and spawns differential, mecanum, suspension, and IMU controllers accordingly.
- **rsp.launch.py**: Launches the robot description with state publisher and optionally selects skid vs. mecanum kinematics.
- **joystick.launch.py**: Sets up joystick input and controller topics for teleoperation.
- **launch_nav.launch.py**, **localization.launch.py**, **lidar_slam2D.launch.py**, **lidar_slam3D.launch.py**, **slam_only.launch.py**: Navigation and SLAM pipelines for different sensing setups.
- **launch_gpt.launch.py**: Starts the ChatGPT velocity pipeline alongside the speech-to-text topic bridge.

## Building
1. Install ROS 2 and `colcon`. Ensure dependencies such as `rclcpp`, `geometry_msgs`, `sensor_msgs`, `trajectory_msgs`, `nav_msgs`, `controller_manager`, and `pluginlib` are available (see `package.xml`).
2. From your workspace root containing `src/kridtbot`, run:
   ```bash
   colcon build --packages-select kridtbot
   ```
3. Source the overlay (e.g., `source install/setup.bash`).

### External drivers and related packages
Kridtbot integrates with several external ROS packages and drivers for hardware and sensing support. Clone and build these alongside this package when bringing up a full system:
- https://github.com/2b-t/myactuator_rmd_ros
- https://github.com/dheera/ros-imu-bno055
- https://github.com/bob-ros2/voskros
- https://github.com/ros-perception/pointcloud_to_laserscan
- https://github.com/odriverobotics/ros_odrive
- https://github.com/Livox-SDK/livox_ros_driver2
- https://github.com/luxonis/depthai-ros
- https://github.com/Crelle490/linear_actuator_calibration_kridtbot
- https://github.com/Crelle490/bno055_imu_ros2_hardware_interface
- https://github.com/nich1157/suspension_controller

## Running examples
- **Simulation with controllers** (Gazebo):
  ```bash
  ros2 launch kridtbot launch_kridtbot.launch.py use_sim:=true
  ```
- **Hardware controller bring-up**:
  ```bash
  ros2 launch kridtbot launch_kridtbot.launch.py use_sim:=false
  ```
- **Drive a square path with mecanum controller**:
  ```bash
  ros2 run kridtbot square_driver
  ```
- **Send joystick-based suspension positions**:
  ```bash
  ros2 run kridtbot linear_position_pub --ros-args -p axis_index:=4 -p min_value:=-0.005 -p max_value:=0.10
  ```
- **Toggle suspension controller with a joystick button**:
  ```bash
  ros2 run kridtbot switch_node
  ```
- **Use ChatGPT velocity commands** (requires `OPENAI_API_KEY`):
  ```bash
  export OPENAI_API_KEY=your_key
  ros2 launch kridtbot launch_gpt.launch.py
  ```

## Development notes
- The package targets ROS 2 C++14 and enables `-Wall -Wextra -Wpedantic` warnings via CMake.
- Vosk speech-to-text include/library paths are parametrized near the top of `CMakeLists.txt` for future integration.
- Launch files install to `share/kridtbot/launch`, and custom nodes install to `lib/kridtbot` for standard ROS 2 discovery.

## License
See [LICENSE.md](LICENSE.md) for licensing details.
