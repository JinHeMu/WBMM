cmake_minimum_required(VERSION 3.10)
project(wbmm_planner_ros)

# Planning/ESDF/FK are latency-sensitive; preserve an explicit caller build type.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wbmm_core REQUIRED)
find_package(wbmm_environment REQUIRED)
find_package(wbmm_collision REQUIRED)
find_package(wbmm_pinocchio REQUIRED)
find_package(wbmm_planner REQUIRED)
find_package(wbmm_planning_msgs REQUIRED)

# Thin ROS wrapper. Everything algorithmic is in wbmm_planner, which stays
# ROS-free; this package only owns parameters and the ROS interfaces.
add_executable(${PROJECT_NAME}_node src/wbmm_planner_node.cpp)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp std_msgs tf2_ros geometry_msgs nav_msgs sensor_msgs tf2 Eigen3
  wbmm_core wbmm_environment wbmm_collision wbmm_pinocchio
  wbmm_planner wbmm_planning_msgs)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
# End-to-end smoke check. It needs a running node, so it is installed for manual
# use rather than registered as an ament test.
install(PROGRAMS test/planner_integration_check.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
