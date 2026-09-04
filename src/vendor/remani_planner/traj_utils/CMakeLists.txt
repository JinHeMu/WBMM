cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mm_config REQUIRED)

# Generate custom messages (ROS2 style)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DataDisp.msg
  msg/PolyTraj.msg
  msg/Assignment.msg
  msg/WholeBodyGoal.msg
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

add_library(traj_utils_core SHARED
  src/main.cpp
)

target_include_directories(traj_utils_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(traj_utils_core PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  mm_config
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(traj_utils_core PUBLIC "${cpp_typesupport_target}")

install(TARGETS traj_utils_core
  EXPORT export_traj_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_traj_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  rosidl_default_runtime
  mm_config
)

ament_package()
