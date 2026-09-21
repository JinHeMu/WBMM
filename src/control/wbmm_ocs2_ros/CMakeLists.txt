cmake_minimum_required(VERSION 3.10)
project(wbmm_ocs2_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rosidl_generator_py still uses the legacy FindPythonInterp module. Some
# OCS2/Pinocchio dependency chains pre-populate PYTHON_EXECUTABLE as empty,
# so resolve an interpreter explicitly before interface generation.
if(NOT PYTHON_EXECUTABLE)
  find_program(PYTHON_EXECUTABLE NAMES python3 python)
endif()
if(NOT PYTHON_EXECUTABLE)
  message(FATAL_ERROR "Python interpreter is required for ROS interfaces.")
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED)

# 关键：Boost 必须在所有 ocs2_* 之前
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  thread
  log
  log_setup
  chrono
  date_time
  atomic
)

find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(wbmm_ocs2 REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(quadrotor_msgs REQUIRED)

# ---- Generated interfaces for dual-reference task phase control ----
# Some mixed Python/Pinocchio dependency chains define PYTHON_SOABI with a
# leading dot. ROS's FindPythonExtra expects it without the dot; otherwise
# the generated Python extension becomes "..cpython-...so" and is unimportable.
if(DEFINED PYTHON_SOABI)
  string(REGEX REPLACE "^\\." "" PYTHON_SOABI "${PYTHON_SOABI}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskPhaseState.msg"
  "srv/SetTaskPhase.srv"
  DEPENDENCIES std_msgs
)

set(common_deps
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  interactive_markers
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_ros_interfaces
  wbmm_ocs2
  ocs2_msgs
  std_srvs
  ocs2_self_collision_visualization
  quadrotor_msgs
)

include_directories(include)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- MPC node ----
add_executable(wbmm_mpc_node src/WbmmMpcNode.cpp)
ament_target_dependencies(wbmm_mpc_node ${common_deps})
target_link_libraries(wbmm_mpc_node
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
  "${cpp_typesupport_target}"
)

# ---- MRT / Gazebo bridge node ----
add_executable(wbmm_mrt_node
src/WbmmMrtNode.cpp
src/WbmmVisualization.cpp
)
ament_target_dependencies(wbmm_mrt_node ${common_deps})
target_link_libraries(wbmm_mrt_node
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
  "${cpp_typesupport_target}"
)

# ---- Target publisher ----
add_executable(wbmm_target_node src/WbmmTargetNode.cpp)
ament_target_dependencies(wbmm_target_node ${common_deps})
target_link_libraries(wbmm_target_node
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
)

# ---- REMANI polynomial trajectory -> OCS2 whole-body reference ----
add_executable(remani_to_ocs2_reference_bridge
  src/remani_to_ocs2_reference_bridge.cpp
)
ament_target_dependencies(remani_to_ocs2_reference_bridge ${common_deps})
target_link_libraries(remani_to_ocs2_reference_bridge
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
)



# ---- Install ----
install(TARGETS
  wbmm_mpc_node
  wbmm_mrt_node
  wbmm_target_node
  remani_to_ocs2_reference_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config rviz scripts include
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/send_target.py
  scripts/set_task_phase.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
