cmake_minimum_required(VERSION 3.5)
project(kridtbot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Vosk paths
set(VOSK_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../../vosk-api/cpp")
set(VOSK_LIB_DIR "${CMAKE_SOURCE_DIR}/../../vosk-api/cpp/build")

# find dependencies
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

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${VOSK_INCLUDE_DIR}
)


add_executable(linear_position_pub nodes/linear_position_pub.cpp)

add_executable(gpt_pose_extractor nodes/gptPoseExtractor.cpp)

ament_target_dependencies(linear_position_pub
  rclcpp
  sensor_msgs
  std_msgs
)

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


install(
  DIRECTORY config description launch worlds meshes nodes
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

