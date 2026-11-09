cmake_minimum_required(VERSION 3.5)
project(slam_gmapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(openslam_gmapping REQUIRED)

include_directories(include)

add_executable(slam_gmapping src/slam_gmapping.cpp)

ament_target_dependencies(slam_gmapping
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  openslam_gmapping)

install(TARGETS slam_gmapping
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/odom_drift.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
