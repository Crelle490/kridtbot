cmake_minimum_required(VERSION 3.5)
project(kridtbot)

# C/C++ standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Vosk paths
set(VOSK_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../../vosk-api/cpp")
set(VOSK_LIB_DIR "${CMAKE_SOURCE_DIR}/../../vosk-api/cpp/build")

# Vosk paths
#set(VOSK_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../../vosk-api/cpp")
#set(VOSK_LIB_DIR "${CMAKE_SOURCE_DIR}/../../vosk-api/cpp/build")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find dependencies

# Includes
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${VOSK_INCLUDE_DIR}
)

# Executables
add_executable(linear_position_pub nodes/linear_position_pub.cpp)
add_executable(square_driver nodes/square_driver.cpp)
add_executable(FKtest nodes/FKtest.cpp)
add_executable(imu_covariance_node nodes/imu_covariance.cpp)
add_executable(switch_node nodes/SwitchController.cpp)
add_executable(gpt_pose_extractor nodes/gptPoseExtractor.cpp)

# Target dependencies
ament_target_dependencies(imu_covariance_node rclcpp sensor_msgs)
ament_target_dependencies(linear_position_pub rclcpp sensor_msgs std_msgs)
ament_target_dependencies(square_driver rclcpp geometry_msgs)
ament_target_dependencies(switch_node rclcpp sensor_msgs controller_manager_msgs)

ament_target_dependencies(linear_position_pub
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(square_driver
rclcpp
geometry_msgs
)

ament_target_dependencies(FKtest rclcpp std_msgs)
ament_target_dependencies(gpt_pose_extractor rclcpp std_msgs geometry_msgs)

# Link libraries
target_link_libraries(gpt_pose_extractor curl)

# Install targets
install(TARGETS
  linear_position_pub
  imu_covariance_node
  square_driver
  FKtest
  switch_node
  gpt_pose_extractor)

ament_target_dependencies(gpt_pose_extractor 
  rclcpp
  std_msgs 
  geometry_msgs)

# Link libraries
target_link_libraries(gpt_pose_extractor curl)

install(TARGETS 
  linear_position_pub
  gpt_pose_extractor
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gpt_pose_extractor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Install folders
install(
  DIRECTORY config description launch worlds meshes nodes
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

