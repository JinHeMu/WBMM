cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate messages
set(msg_files
  msg/AuxCommand.msg
  msg/Corrections.msg
  msg/Gains.msg
  msg/OutputData.msg
  msg/PositionCommand.msg
  msg/PPROutputData.msg
  msg/Serial.msg
  msg/SO3Command.msg
  msg/StatusData.msg
  msg/TRPYCommand.msg
  msg/Odometry.msg
  msg/PolynomialTrajectory.msg
  msg/PolynomialTraj.msg
  msg/PolynomialMatrix.msg
  msg/TakeoffLand.msg
  msg/Px4ctrlDebug.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# Helper libraries
add_library(decode_msgs SHARED src/decode_msgs.cpp)
add_library(encode_msgs SHARED src/encode_msgs.cpp)

target_include_directories(decode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})

target_include_directories(encode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})

target_link_libraries(decode_msgs
  rclcpp::rclcpp
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES})

target_link_libraries(encode_msgs
  rclcpp::rclcpp
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES})

rosidl_target_interfaces(decode_msgs ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(encode_msgs ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
  nav_msgs
)

install(TARGETS decode_msgs encode_msgs
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

ament_package()
