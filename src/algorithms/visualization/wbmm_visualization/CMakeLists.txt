cmake_minimum_required(VERSION 3.8)
project(wbmm_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/whole_body_kinematics.cpp
  src/contract.cpp
  src/markers.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC pinocchio::pinocchio)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC
  Eigen3 rclcpp geometry_msgs nav_msgs std_msgs trajectory_msgs
  visualization_msgs)

add_executable(wbmm_viz_node src/wbmm_viz_node.cpp)
target_link_libraries(wbmm_viz_node ${PROJECT_NAME}_core)
ament_target_dependencies(wbmm_viz_node rclcpp sensor_msgs std_msgs)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS wbmm_viz_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_visualizer test/test_visualizer.cpp)
  target_link_libraries(test_visualizer ${PROJECT_NAME}_core)
  ament_target_dependencies(test_visualizer rclcpp)
  target_compile_definitions(test_visualizer PRIVATE
    WBMM_VIZ_WORKSPACE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME}_core)
  ament_target_dependencies(test_kinematics rclcpp)
  target_compile_definitions(test_kinematics PRIVATE
    WBMM_VIZ_WORKSPACE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Eigen3 pinocchio geometry_msgs nav_msgs std_msgs
  trajectory_msgs visualization_msgs)
ament_package()
