cmake_minimum_required(VERSION 3.10)
project(diy_sim)

# Find required packages
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Install directories
install(
  DIRECTORY
    models
    worlds
    config
  DESTINATION
    share/${PROJECT_NAME}
)

# Install launch files
install(
  FILES
    sim_betsybot.launch.py
    sim_competition.launch.py
    sim_simple_loop.launch.py
  DESTINATION
    share/${PROJECT_NAME}
)

# Crazy Ball Plugin
#add_library(sim-crazy-ball-plugin SHARED
#  plugins/crazy_ball_plugin.cpp
#)

#target_include_directories(sim-crazy-ball-plugin PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

#target_link_libraries(sim-crazy-ball-plugin
#  gz-sim8::gz-sim8
#  gz-plugin2::gz-plugin2
#)

# Install plugins
#install(
#  TARGETS
#    sim-crazy-ball-plugin
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# Install Python modules if any
# ament_python_install_package(${PROJECT_NAME})

# Add custom target to copy models to Gazebo model path during build
#add_custom_target(copy_models ALL
#  COMMAND ${CMAKE_COMMAND} -E make_directory $ENV{HOME}/.gz/models/${PROJECT_NAME}
#  COMMAND ${CMAKE_COMMAND} -E copy_directory
#    ${CMAKE_CURRENT_SOURCE_DIR}/models/ackermann_car
#    $ENV{HOME}/.gz/models/ackermann_car
#)

# Mark cpp executables for installation
# install(TARGETS
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python executables
install(
  PROGRAMS
    scripts/sim_telemetry.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(
  rclpy
  geometry_msgs
  nav_msgs
)

# Package specific configuration
ament_package()
