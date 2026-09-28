cmake_minimum_required(VERSION 3.10)
project(wbmm_reference_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(wbmm_planning_msgs REQUIRED)

# Sampling and validation of the planned trajectory. Kept in its own library so
# it can be tested without a running node or a clock.
add_library(${PROJECT_NAME}_sampler src/trajectory_sampler.cpp)
target_include_directories(${PROJECT_NAME}_sampler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_sampler PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_sampler PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}_sampler wbmm_planning_msgs)

add_executable(${PROJECT_NAME}_node src/wbmm_reference_bridge_node.cpp)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_sampler)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp std_msgs tf2 tf2_ros std_srvs Eigen3 ocs2_core ocs2_msgs ocs2_ros_interfaces
  wbmm_planning_msgs)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_sampler test/test_trajectory_sampler.cpp)
  target_link_libraries(test_trajectory_sampler ${PROJECT_NAME}_sampler)
  ament_target_dependencies(test_trajectory_sampler wbmm_planning_msgs)
endif()

ament_package()
