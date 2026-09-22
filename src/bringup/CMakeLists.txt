cmake_minimum_required(VERSION 3.8)
project(tracer_jaka_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Launch files are backend-agnostic and live directly in launch/.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY config rviz
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
  ament_add_pytest_test(
    common_launch
    test/test_common_launch.py
    TIMEOUT 30
    APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/common_launch_logs
  )
  # Register this safety/config contract explicitly.  On this workspace the
  # system pytest is usable with third-party plugin autoload disabled, while
  # ament_cmake_pytest's import probe is contaminated by an incompatible user
  # anyio plugin and would silently skip the test.
  add_test(
    NAME force_control_configs
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/force_config_logs"
      "${Python3_EXECUTABLE}" -m pytest -q
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_force_control_configs.py"
  )
  set_tests_properties(force_control_configs PROPERTIES TIMEOUT 30)
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
