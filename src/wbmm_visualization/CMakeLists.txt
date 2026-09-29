cmake_minimum_required(VERSION 3.10)
project(wbmm_visualization)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wbmm_core REQUIRED)
# Display consumes the shared URDF description and the shared Pinocchio
# kinematics; it no longer parses URDF or computes FK itself, so the ROS urdf
# package is not a dependency any more.
find_package(wbmm_robot_model REQUIRED)
find_package(wbmm_pinocchio REQUIRED)
find_package(wbmm_planner_ros REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
add_library(${PROJECT_NAME} src/robot_visual_model.cpp src/trajectory_display.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} visualization_msgs wbmm_planner_ros ocs2_msgs)
target_link_libraries(${PROJECT_NAME}
  wbmm_core::wbmm_core
  wbmm_robot_model::wbmm_robot_model
  wbmm_pinocchio::wbmm_pinocchio
  Eigen3::Eigen)
add_executable(trajectory_visualizer src/visualization_node.cpp)
target_link_libraries(trajectory_visualizer ${PROJECT_NAME})
ament_target_dependencies(trajectory_visualizer rclcpp)
target_compile_options(trajectory_visualizer PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS trajectory_visualizer RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS test/ros_display_check.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_visualization test/test_visualization.cpp)
  target_link_libraries(test_visualization ${PROJECT_NAME})
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 wbmm_core wbmm_robot_model wbmm_pinocchio visualization_msgs wbmm_planner_ros ocs2_msgs)
ament_package()
