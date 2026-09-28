cmake_minimum_required(VERSION 3.8)
project(wall_follower_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(plugin_dependencies
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(wall_follower_controller_plugin SHARED
  src/wall_follower_controller.cpp
)
target_include_directories(wall_follower_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(
  wall_follower_controller_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(wall_follower_controller_plugin ${plugin_dependencies})

pluginlib_export_plugin_description_file(
  nav2_core wall_follower_controller_plugins.xml)

install(TARGETS wall_follower_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES wall_follower_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(wall_follower_controller_plugin)
ament_export_dependencies(${plugin_dependencies})

ament_package()
