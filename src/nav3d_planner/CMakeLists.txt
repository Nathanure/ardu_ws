cmake_minimum_required(VERSION 3.10)
project(nav3d_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Install Python scripts directly (no Python package needed)
install(PROGRAMS
  scripts/planner_3d_node.py
  scripts/goal_publisher.py
  scripts/mission_planner.py
  scripts/ardupilot_interface.py
  scripts/odom_to_pose_node.py
  scripts/odom_to_tf_node.py
  scripts/pointcloud_fixer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, and rviz
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# If you want to build C++ nodes later, add them here
# add_executable(planner_3d_cpp src/planner_3d.cpp)
# ament_target_dependencies(planner_3d_cpp rclcpp ...)
# install(TARGETS planner_3d_cpp DESTINATION lib/${PROJECT_NAME})

ament_package()