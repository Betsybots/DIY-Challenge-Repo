cmake_minimum_required(VERSION 3.10)
project(diy_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/trigger_map_relocalize.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
