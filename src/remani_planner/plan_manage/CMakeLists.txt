cmake_minimum_required(VERSION 3.8)
project(remani_planner)

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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# Dependencies on our packages
find_package(plan_env REQUIRED)
find_package(mm_config REQUIRED)
find_package(path_searching REQUIRED)
find_package(traj_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(quadrotor_msgs REQUIRED)

# Visualization utility library
add_library(vis_utils SHARED
  src/planning_visualization.cpp
)

target_include_directories(vis_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(vis_utils PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  traj_utils
)

# Main planner node
add_executable(remani_planner_node
  src/remani_planner_node.cpp
  src/remani_replan_fsm.cpp
  src/planner_manager.cpp
)

target_include_directories(remani_planner_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(remani_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  traj_utils
  quadrotor_msgs
)

target_link_libraries(remani_planner_node
  plan_env::plan_env
  mm_config::mm_config
  path_searching::path_searching
  traj_opt::traj_opt
  traj_utils::traj_utils_core
  vis_utils
)

install(TARGETS remani_planner_node vis_utils
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/remani_simulator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME remani_simulator
)

ament_package()
