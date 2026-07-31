cmake_minimum_required(VERSION 3.8)
project(diy_ndt_localization)

# Require C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Release build for performance on Jetson — NDT-OMP is CPU-heavy
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ── NDT localizer node ──────────────────────────────────────────────────────
add_executable(ndt_localizer_node src/ndt_localizer_node.cpp)

ament_target_dependencies(ndt_localizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  ndt_omp_ros2
)

target_link_libraries(ndt_localizer_node ${PCL_LIBRARIES})

install(TARGETS ndt_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
