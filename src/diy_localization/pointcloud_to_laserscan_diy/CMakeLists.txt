cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_laserscan_diy)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_flattener_node src/scan_flattener_node.cpp)
ament_target_dependencies(scan_flattener_node rclcpp sensor_msgs)

install(TARGETS scan_flattener_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/bag_replay_bridge.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
