cmake_minimum_required(VERSION 3.8)
project(ta_wbmp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(wbmm_visualization REQUIRED)
find_package(whole_body_force_control REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(ta_wbmp_core
  src/cost.cpp
  src/extensions.cpp
  src/planner.cpp
  src/task_trajectory.cpp)
target_include_directories(ta_wbmp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ta_wbmp_core pinocchio::pinocchio yaml-cpp)
ament_target_dependencies(ta_wbmp_core PUBLIC Eigen3)

add_executable(ta_wbmp_demo_node src/demo_node.cpp)
target_link_libraries(ta_wbmp_demo_node ta_wbmp_core
  wbmm_visualization::wbmm_visualization_core)
ament_target_dependencies(ta_wbmp_demo_node
  geometry_msgs nav_msgs ocs2_msgs rclcpp std_msgs trajectory_msgs traj_utils
  visualization_msgs wbmm_visualization)

add_executable(ta_wbmp_scenario_runner src/scenario_runner.cpp)
target_link_libraries(ta_wbmp_scenario_runner ta_wbmp_core)
ament_target_dependencies(ta_wbmp_scenario_runner Eigen3)

add_executable(ta_wbmp_execution_coordinator_node
  src/execution_coordinator_node.cpp)
target_link_libraries(ta_wbmp_execution_coordinator_node ta_wbmp_core)
ament_target_dependencies(ta_wbmp_execution_coordinator_node
  geometry_msgs ocs2_msgs rclcpp std_msgs std_srvs tf2 tf2_ros traj_utils
  whole_body_force_control)

install(TARGETS ta_wbmp_core ta_wbmp_demo_node ta_wbmp_scenario_runner
  ta_wbmp_execution_coordinator_node
  EXPORT export_ta_wbmp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESIGN.md TASK_SCHEMA.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ta_wbmp test/test_planner.cpp)
  target_link_libraries(test_ta_wbmp ta_wbmp_core)
  target_compile_definitions(test_ta_wbmp PRIVATE
    TA_WBMP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    TA_WBMP_WORKSPACE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
endif()

ament_export_targets(export_ta_wbmp HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 pinocchio yaml-cpp)
ament_package()
