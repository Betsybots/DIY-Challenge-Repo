cmake_minimum_required(VERSION 3.8)
project(diy_zone_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(zone_nav_manager_node src/zone_nav_manager_node.cpp)

ament_target_dependencies(zone_nav_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  rcl_interfaces
)

add_executable(lidar_odom_gate_node src/lidar_odom_gate_node.cpp)

ament_target_dependencies(lidar_odom_gate_node
  rclcpp
  std_msgs
  nav_msgs
)

install(TARGETS zone_nav_manager_node lidar_odom_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
