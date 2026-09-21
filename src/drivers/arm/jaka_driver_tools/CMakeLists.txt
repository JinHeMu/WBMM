cmake_minimum_required(VERSION 3.8)
project(jaka_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(jaka_sdk_vendor REQUIRED)
get_filename_component(JAKA_SDK_LIBRARY_DIR
  "${jaka_sdk_vendor_LIBRARIES}" DIRECTORY)

include_directories(
  ${jaka_sdk_vendor_INCLUDE_DIRS}
)

add_executable(jaka_edg_node    src/jaka_edg_node.cpp)
add_executable(jaka_login       src/jaka_login.cpp)
add_executable(jaka_logout      src/jaka_logout.cpp)
add_executable(jaka_tcp_sensor  src/jaka_tcp_sensor.cpp)

ament_target_dependencies(jaka_edg_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
)

target_link_libraries(jaka_edg_node
  ${jaka_sdk_vendor_LIBRARIES}
)
target_link_libraries(jaka_login
  ${jaka_sdk_vendor_LIBRARIES}
)
target_link_libraries(jaka_logout
  ${jaka_sdk_vendor_LIBRARIES}
)
target_link_libraries(jaka_tcp_sensor
  ${jaka_sdk_vendor_LIBRARIES}
)

set_target_properties(
  jaka_edg_node
  jaka_login
  jaka_logout
  jaka_tcp_sensor
  PROPERTIES
    BUILD_RPATH "${JAKA_SDK_LIBRARY_DIR}"
    INSTALL_RPATH "$ORIGIN/.."
)

install(TARGETS
  jaka_edg_node
  jaka_login
  jaka_logout
  jaka_tcp_sensor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  jaka_sdk_vendor
)

ament_package()
