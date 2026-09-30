cmake_minimum_required(VERSION 3.8)
project(mapless_wall_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mapless_wall_follower_node
  src/mapless_wall_follower_node.cpp
)
ament_target_dependencies(mapless_wall_follower_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS mapless_wall_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
