cmake_minimum_required(VERSION 3.8)
project(wipe_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(wbmm_visualization REQUIRED)
find_package(whole_body_force_control REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(wipe_planner_core src/planner.cpp)
target_include_directories(wipe_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wipe_planner_core pinocchio::pinocchio yaml-cpp)
ament_target_dependencies(wipe_planner_core PUBLIC Eigen3)

add_executable(wipe_planner_node src/node.cpp)
target_link_libraries(wipe_planner_node wipe_planner_core
  wbmm_visualization::wbmm_visualization_core)
ament_target_dependencies(wipe_planner_node
  ament_index_cpp geometry_msgs nav_msgs ocs2_msgs quadrotor_msgs rclcpp
  sensor_msgs std_msgs std_srvs tf2_ros traj_utils trajectory_msgs
  visualization_msgs wbmm_visualization whole_body_force_control)

add_executable(wipe_plan_preview_node src/preview_node.cpp)
target_link_libraries(wipe_plan_preview_node wipe_planner_core)
ament_target_dependencies(wipe_plan_preview_node
  geometry_msgs nav_msgs rclcpp std_msgs visualization_msgs)

add_executable(wipe_future_task_batch src/future_task_batch.cpp)
target_link_libraries(wipe_future_task_batch wipe_planner_core)
ament_target_dependencies(wipe_future_task_batch Eigen3)

install(TARGETS wipe_planner_core wipe_planner_node wipe_plan_preview_node
  wipe_future_task_batch
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_differential_drive test/test_differential_drive.cpp)
  target_link_libraries(test_differential_drive wipe_planner_core)
  target_compile_definitions(test_differential_drive PRIVATE
    WIPE_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    WIPE_PLANNER_WORKSPACE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
  # This executable also runs the conditioned full-contact rollout regression,
  # which takes about two minutes on the target development machine.
  set_tests_properties(test_differential_drive PROPERTIES TIMEOUT 240)

  ament_add_gtest(test_frame_transform test/test_frame_transform.cpp)
  target_link_libraries(test_frame_transform wipe_planner_core)

  ament_add_gtest(test_real_front_plan test/test_real_front_plan.cpp)
  target_link_libraries(test_real_front_plan wipe_planner_core)
  target_compile_definitions(test_real_front_plan PRIVATE
    WIPE_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    WIPE_PLANNER_WORKSPACE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
  set_tests_properties(test_real_front_plan PROPERTIES TIMEOUT 60)

  ament_add_gtest(test_mujoco_table_plan test/test_mujoco_table_plan.cpp)
  target_link_libraries(test_mujoco_table_plan wipe_planner_core)
  target_compile_definitions(test_mujoco_table_plan PRIVATE
    WIPE_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    WIPE_PLANNER_WORKSPACE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
  set_tests_properties(test_mujoco_table_plan PROPERTIES TIMEOUT 90)
endif()

ament_export_include_directories(include)
ament_export_libraries(wipe_planner_core)
ament_package()
