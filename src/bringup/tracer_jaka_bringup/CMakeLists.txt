cmake_minimum_required(VERSION 3.8)
project(tracer_jaka_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/arm_pose_publisher.py
  scripts/odom_to_map_relay.py
  scripts/readiness_check.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_pytest_test(
    localized_real_safety_gate
    test/test_localized_real_safety_gate.py
    TIMEOUT 30
  )
  add_test(
    NAME force_control_real_safety_gate
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_force_control_real_safety_gate.py"
  )
  set_tests_properties(force_control_real_safety_gate PROPERTIES
    TIMEOUT 30
    ENVIRONMENT "ROS_LOG_DIR=/tmp/wbmm_force_control_safety_test_logs"
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
