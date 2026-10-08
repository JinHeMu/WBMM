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
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(wbmm_core REQUIRED)
find_package(wbmm_pinocchio REQUIRED)
find_package(wbmm_robot_model REQUIRED)
find_package(wbmm_ros_interfaces REQUIRED)

# 力控算法核心：力处理 + 力控控制器，不包含机器人模型/运动学/ROS 转换。
add_library(${PROJECT_NAME}_core
  src/controllers.cpp
  src/force_processor.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  Eigen3::Eigen
  wbmm_core::wbmm_core)

add_executable(whole_body_force_control_node
  src/node.cpp
  src/node_config.cpp
  src/node_ros_io.cpp)
target_link_libraries(whole_body_force_control_node
  ${PROJECT_NAME}_core
  wbmm_pinocchio::wbmm_pinocchio
  wbmm_ros_interfaces::wbmm_ros_interfaces
  wbmm_core::wbmm_core)
ament_target_dependencies(whole_body_force_control_node
  geometry_msgs ocs2_msgs rclcpp std_msgs std_srvs)

add_executable(force_sensor_processor_node
  src/force_sensor_processor_node.cpp)
target_link_libraries(force_sensor_processor_node
  ${PROJECT_NAME}_core
  wbmm_ros_interfaces::wbmm_ros_interfaces
  wbmm_core::wbmm_core)
ament_target_dependencies(force_sensor_processor_node
  geometry_msgs rclcpp std_msgs std_srvs tf2_ros)

add_executable(virtual_wrench_node src/virtual_wrench_node.cpp)
target_link_libraries(virtual_wrench_node Eigen3::Eigen)
ament_target_dependencies(virtual_wrench_node geometry_msgs rclcpp rcl_interfaces tf2_ros)

find_package(X11 REQUIRED)
add_executable(keyboard_wrench_node src/keyboard_wrench_node.cpp)
target_include_directories(keyboard_wrench_node PRIVATE ${X11_INCLUDE_DIR})
target_link_libraries(keyboard_wrench_node ${X11_LIBRARIES})
ament_target_dependencies(keyboard_wrench_node geometry_msgs rclcpp)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS whole_body_force_control_node force_sensor_processor_node virtual_wrench_node keyboard_wrench_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
# Deployment profiles are centralized in tracer_jaka_bringup/config/.
install(PROGRAMS
  scripts/whole_body_force_control_test.py
  scripts/fake_wrench_sequence.py
  scripts/axis_wrench_sequence.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  ament_add_gtest(test_force_control test/test_force_control.cpp)
  target_link_libraries(test_force_control
    ${PROJECT_NAME}_core
    wbmm_pinocchio::wbmm_pinocchio
    wbmm_robot_model::wbmm_robot_model
    wbmm_ros_interfaces::wbmm_ros_interfaces)
  ament_target_dependencies(test_force_control ament_index_cpp)
  # Manual test against an already running isolated MuJoCo force_mpc launch.
  # Never register this as an automatic hardware-facing CTest.
  add_executable(force_mpc_integration_probe test/force_mpc_integration_probe.cpp)
  target_link_libraries(force_mpc_integration_probe
    wbmm_pinocchio::wbmm_pinocchio wbmm_ros_interfaces::wbmm_ros_interfaces
    wbmm_robot_model::wbmm_robot_model)
  ament_target_dependencies(force_mpc_integration_probe
    ament_index_cpp rclcpp geometry_msgs ocs2_msgs std_msgs std_srvs)
  install(TARGETS force_mpc_integration_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
  # 测试只依赖 package share 里的 URDF；fallback 仅用于未安装描述的源码工作区。
  target_compile_definitions(test_force_control PRIVATE
    WHOLE_BODY_FORCE_CONTROL_TEST_URDF_FALLBACK="${CMAKE_CURRENT_SOURCE_DIR}/../../robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Eigen3 wbmm_pinocchio wbmm_ros_interfaces wbmm_core)
ament_package()
