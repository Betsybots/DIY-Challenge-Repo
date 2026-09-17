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
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

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

# ── nav2_core::Controller plugin ─────────────────────────────────────────────
# Real Nav2 controller_server plugin (loaded via pluginlib, not a standalone
# executable) -- see include/diy_motion_planner/pure_pursuit_controller.hpp
# for the architectural notes on how this differs from
# pure_pursuit_motion_planner_node above, and the gap around who invokes it
# (nothing does yet -- no bt_navigator, no action client). Both the old
# standalone node and this new plugin currently coexist; nothing has been
# removed.
set(plugin_dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

add_library(diy_motion_planner_plugin SHARED
  src/pure_pursuit_controller.cpp
)
target_compile_definitions(diy_motion_planner_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(diy_motion_planner_plugin ${plugin_dependencies})

install(TARGETS diy_motion_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(diy_motion_planner_plugin)
ament_export_dependencies(${plugin_dependencies})

pluginlib_export_plugin_description_file(nav2_core diy_motion_planner_plugins.xml)

ament_package()
