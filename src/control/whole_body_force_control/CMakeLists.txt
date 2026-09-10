cmake_minimum_required(VERSION 3.8)
project(whole_body_force_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core
  src/controllers.cpp
  src/whole_body_kinematics.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core pinocchio::pinocchio)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC Eigen3)

add_executable(whole_body_force_control_node src/node.cpp)
target_link_libraries(whole_body_force_control_node ${PROJECT_NAME}_core)
ament_target_dependencies(whole_body_force_control_node
  geometry_msgs ocs2_msgs rclcpp std_msgs std_srvs tf2_ros)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS whole_body_force_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/whole_body_force_control_test.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_force_control test/test_force_control.cpp)
  target_link_libraries(test_force_control ${PROJECT_NAME}_core)
  target_compile_definitions(test_force_control PRIVATE
    WHOLE_BODY_FORCE_CONTROL_WORKSPACE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Eigen3 pinocchio)
ament_package()
