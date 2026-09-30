cmake_minimum_required(VERSION 3.8)
project(wbmm_localization)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nlohmann_json REQUIRED)

# Python is used only for ROS launch composition.
ament_python_install_package(wbmm_localization_launch
  PACKAGE_DIR launch/wbmm_localization_launch)
add_library(readiness_core src/readiness.cpp)
target_compile_features(readiness_core PUBLIC cxx_std_17)
target_include_directories(readiness_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(readiness_core PUBLIC nlohmann_json::nlohmann_json)
add_library(readiness_nodes src/readiness_node.cpp src/wait_node.cpp)
target_link_libraries(readiness_nodes PUBLIC readiness_core)
ament_target_dependencies(readiness_nodes PUBLIC rclcpp sensor_msgs nav_msgs std_msgs tf2_ros tf2)
add_executable(localization_readiness src/localization_readiness_main.cpp)
target_link_libraries(localization_readiness readiness_nodes)
add_executable(wait_for_localization src/wait_for_localization_main.cpp)
target_link_libraries(wait_for_localization readiness_nodes)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(readiness_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(readiness_nodes PRIVATE -Wall -Wextra -Wpedantic)
endif()
install(TARGETS readiness_core readiness_nodes localization_readiness wait_for_localization
  EXPORT localization_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(localization_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs nav_msgs std_msgs tf2_ros tf2 nlohmann_json)
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(geometry_msgs REQUIRED)
  ament_add_gtest(readiness_core_tests test/test_readiness.cpp)
  target_link_libraries(readiness_core_tests readiness_core)
  ament_add_gtest(readiness_ros test/test_readiness_ros.cpp TIMEOUT 20)
  target_link_libraries(readiness_ros readiness_nodes)
  ament_target_dependencies(readiness_ros geometry_msgs)
  set_tests_properties(readiness_ros PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=173;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME localization_tests
    COMMAND ${CMAKE_COMMAND} -E env
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/launch:$ENV{PYTHONPATH}
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs
      ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test)
  set_tests_properties(localization_tests PROPERTIES TIMEOUT 60)
endif()
ament_package()
