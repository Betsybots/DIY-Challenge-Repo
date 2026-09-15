cmake_minimum_required(VERSION 3.8)
project(diy_motion_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pd_motion_planner_node src/pd_motion_planner_node.cpp)
ament_target_dependencies(pd_motion_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pure_pursuit_motion_planner_node src/pure_pursuit_motion_planner_node.cpp)
ament_target_dependencies(pure_pursuit_motion_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS pd_motion_planner_node pure_pursuit_motion_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
