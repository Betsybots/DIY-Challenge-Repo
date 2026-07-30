cmake_minimum_required(VERSION 3.10)
project(plan_b)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(motion_plan_executor_node src/motion_plan_executor_node.cpp)
ament_target_dependencies(motion_plan_executor_node rclcpp geometry_msgs nav_msgs)
target_link_libraries(motion_plan_executor_node yaml-cpp)

install(TARGETS motion_plan_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
