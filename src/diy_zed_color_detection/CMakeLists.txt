cmake_minimum_required(VERSION 3.16)
project(diy_zed_color_detection CXX)

# Targets ROS 2 Humble (Ubuntu 22.04).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Core detector (OpenCV only, no ROS).
add_library(color_detector_core STATIC src/color_detector.cpp)
target_include_directories(color_detector_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(color_detector_core PUBLIC ${OpenCV_LIBS})
set_target_properties(color_detector_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ROS node as a composable component, plus a standalone executable.
set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  message_filters
  cv_bridge
  image_transport
)
add_library(color_detector_component SHARED src/color_detector_node.cpp)
target_link_libraries(color_detector_component color_detector_core)
ament_target_dependencies(color_detector_component ${dependencies})
rclcpp_components_register_node(color_detector_component
  PLUGIN "zcd::ColorDetectorNode"
  EXECUTABLE color_detector_node)

install(TARGETS color_detector_core color_detector_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
