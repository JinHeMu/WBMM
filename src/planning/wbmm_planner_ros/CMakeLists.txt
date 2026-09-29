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
find_package(rosidl_default_generators REQUIRED)
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
find_package(wbmm_robot_model REQUIRED)
find_package(wbmm_planner REQUIRED)

# ROS's legacy Python finder can inherit an empty cached interpreter path.
# Use the interpreter resolved by modern CMake for generated Python bindings.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}" CACHE FILEPATH
  "Python interpreter for ROS interface generation" FORCE)
if(DEFINED PYTHON_SOABI)
  string(REGEX REPLACE "^\\." "" PYTHON_SOABI "${PYTHON_SOABI}")
endif()
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WholeBodyTrajectory.msg"
  "msg/WholeBodyGoal.msg"
  DEPENDENCIES std_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# The algorithmic library remains ROS-free; this package owns the node and messages.
add_executable(${PROJECT_NAME}_node src/wbmm_planner_node.cpp)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp std_msgs tf2_ros geometry_msgs nav_msgs sensor_msgs tf2 Eigen3
  wbmm_core wbmm_environment wbmm_collision wbmm_pinocchio
  wbmm_robot_model wbmm_planner)
target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
# End-to-end smoke check. It needs a running node, so it is installed for manual
# use rather than registered as an ament test.
install(PROGRAMS test/planner_integration_check.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
