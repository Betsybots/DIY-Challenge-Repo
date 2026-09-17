cmake_minimum_required(VERSION 3.8)
project(planning)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

# Nav2 GlobalPlanner plugin, loaded by planner_server via pluginlib.
add_library(planning_astar_plugin SHARED src/a_star_planner_node.cpp)
target_include_directories(planning_astar_plugin PRIVATE include)
ament_target_dependencies(planning_astar_plugin
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS planning_astar_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(planning_astar_plugin)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs tf2 tf2_ros
  nav2_core nav2_costmap_2d nav2_util pluginlib
)

ament_package()
