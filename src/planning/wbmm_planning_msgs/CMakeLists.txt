cmake_minimum_required(VERSION 3.10)
project(wbmm_planning_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# rosidl_generator_py still uses the legacy FindPythonInterp module. Other
# packages in this workspace pre-populate PYTHON_EXECUTABLE as empty, so resolve
# an interpreter explicitly or the Python bindings are silently skipped.
if(NOT PYTHON_EXECUTABLE)
  find_program(PYTHON_EXECUTABLE NAMES python3 python)
endif()
if(NOT PYTHON_EXECUTABLE)
  message(FATAL_ERROR "Python interpreter is required for ROS interfaces.")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WholeBodyTrajectory.msg"
  "msg/WholeBodyGoal.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
