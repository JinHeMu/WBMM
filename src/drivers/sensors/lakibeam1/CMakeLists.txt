cmake_minimum_required(VERSION 3.8)
project(lakibeam1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(scan_assembler_tests test/test_scan_assembler.cpp src/scan_assembler.cpp)
  target_include_directories(scan_assembler_tests PRIVATE include)
  target_compile_features(scan_assembler_tests PRIVATE cxx_std_17)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_scan_node src/lakibeam1_scan.cpp src/scan_assembler.cpp src/remote.cpp)
target_compile_features(${PROJECT_NAME}_scan_node PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME}_scan_node ${ament_LIBRARIES} curl)
ament_target_dependencies(${PROJECT_NAME}_scan_node rclcpp sensor_msgs std_msgs CURL)

if(BUILD_TESTING)
  ament_add_gtest(scan_udp_ros test/test_scan_udp_ros.cpp TIMEOUT 15)
  ament_target_dependencies(scan_udp_ros rclcpp sensor_msgs)
  target_compile_features(scan_udp_ros PRIVATE cxx_std_17)
  target_compile_definitions(scan_udp_ros PRIVATE SCAN_NODE_PATH="$<TARGET_FILE:${PROJECT_NAME}_scan_node>")
  add_dependencies(scan_udp_ros ${PROJECT_NAME}_scan_node)
  set_tests_properties(scan_udp_ros PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=175;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/udp_test_logs")
endif()

install(TARGETS ${PROJECT_NAME}_scan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
