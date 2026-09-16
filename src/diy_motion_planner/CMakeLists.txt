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

# Nav2 Controller plugin, loaded by controller_server via pluginlib.
add_library(pure_pursuit_controller_plugin SHARED src/pure_pursuit_motion_planner_node.cpp)
target_include_directories(pure_pursuit_controller_plugin PRIVATE include)
ament_target_dependencies(pure_pursuit_controller_plugin
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS pd_motion_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS pure_pursuit_controller_plugin
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

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(pure_pursuit_controller_plugin)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs std_msgs visualization_msgs
  tf2 tf2_ros nav2_core nav2_costmap_2d nav2_util pluginlib
)

ament_package()
