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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_executable(linear_position_pub nodes/linear_position_pub.cpp)
add_executable(square_driver nodes/square_driver.cpp)
add_executable(FKtest nodes/FKtest.cpp)
#add_executable(path_publisher nodes/path_publisher.cpp)
add_executable(imu_covariance_node nodes/imu_covariance.cpp)

#ament_target_dependencies(path_publisher rclcpp nav_msgs geometry_msgs)
ament_target_dependencies(imu_covariance_node rclcpp sensor_msgs)

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

install(TARGETS linear_position_pub
  imu_covariance_node
  square_driver
  FKtest
  #path_publisher
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
