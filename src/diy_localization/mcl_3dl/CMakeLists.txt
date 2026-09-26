cmake_minimum_required(VERSION 3.10)
project(mcl_3dl)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  "srv/LoadPCD.srv"
  "srv/ResizeParticle.srv"
  DEPENDENCIES std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8.0 REQUIRED
  COMPONENTS
    core
    filters
    kdtree
)
find_package(Boost REQUIRED COMPONENTS chrono)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Workaround for debian stretch build (https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=894656)
list(REMOVE_ITEM PCL_LIBRARIES
  "vtkGUISupportQt"
  "vtkGUISupportQtOpenGL"
  "vtkGUISupportQtSQL"
  "vtkGUISupportQtWebkit"
  "vtkViewsQt"
  "vtkRenderingQt"
)

# Workaround for the bug in PCL<1.8.1 (https://github.com/PointCloudLibrary/pcl/issues/1406)
remove_definitions(-DDISABLE_LIBUSB-1.0)

# Workaround for PCL-1.7 problem on C++11 (https://github.com/PointCloudLibrary/pcl/issues/619)
# Debug build without optimization causes Segmentation fault.
# Set default build type to Release and add -O1 to CXX_FLAGS_DEBUG.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

# Binary installed pcl provided by Linux distro is built with -march=native
# which causes a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using the binary version.
add_definitions(-DPCL_NO_PRECOMPILE)


add_executable(mcl_3dl_node
  src/cloud_accum.cpp
  src/lidar_measurement_model_beam.cpp
  src/lidar_measurement_model_likelihood.cpp
  src/mcl_3dl.cpp
  src/parameters.cpp
  src/point_types.cpp
)
set_target_properties(mcl_3dl_node PROPERTIES OUTPUT_NAME mcl_3dl)
target_link_libraries(mcl_3dl_node ${Boost_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(mcl_3dl_node
  diagnostic_msgs diagnostic_updater geometry_msgs nav_msgs pcl_conversions pcl_ros rclcpp
  sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros tf2_sensor_msgs
  visualization_msgs)
rosidl_target_interfaces(mcl_3dl_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  ament_add_gtest(test_chunked_kdtree test/src/test_chunked_kdtree.cpp)
  target_link_libraries(test_chunked_kdtree ${PCL_LIBRARIES})
  ament_add_gtest(test_filter test/src/test_filter.cpp)
  ament_add_gtest(test_nd test/src/test_nd.cpp)
  ament_add_gtest(test_pf test/src/test_pf.cpp)
  ament_add_gtest(test_quat test/src/test_quat.cpp)
  ament_add_gtest(test_raycast test/src/test_raycast.cpp)
  target_link_libraries(test_raycast ${PCL_LIBRARIES})
  ament_add_gtest(test_raycast_dda test/src/test_raycast_dda.cpp)
  target_link_libraries(test_raycast_dda ${PCL_LIBRARIES})
  ament_add_gtest(test_state_6dof test/src/test_state_6dof.cpp)
  ament_add_gtest(test_vec3 test/src/test_vec3.cpp)

  set(MCL3DL_UNIT_TESTS
    test_chunked_kdtree test_filter test_nd test_pf test_quat test_raycast
    test_raycast_dda test_state_6dof test_vec3)
  foreach(test_target IN LISTS MCL3DL_UNIT_TESTS)
    target_include_directories(${test_target} PRIVATE include ${PCL_INCLUDE_DIRS})
    target_link_libraries(${test_target} Eigen3::Eigen)
    ament_target_dependencies(${test_target} rclcpp)
  endforeach()
  ament_target_dependencies(test_chunked_kdtree pcl_ros)
  ament_target_dependencies(test_raycast pcl_ros)
  ament_target_dependencies(test_raycast_dda pcl_ros)
endif()


install(TARGETS mcl_3dl_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS launch/test.launch.py DESTINATION share/${PROJECT_NAME}/launch)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
