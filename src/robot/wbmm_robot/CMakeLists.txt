cmake_minimum_required(VERSION 3.10)
project(wbmm_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(wbmm_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/pinocchio_robot_model.cpp
  src/whole_body_kinematics.cpp
  src/wbmm_conversions.cpp
  src/wbmm_ros_conversions.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  pinocchio::pinocchio
  wbmm_core::wbmm_core)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  Eigen3 geometry_msgs ocs2_msgs std_msgs)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wbmm_ros_conversions test/test_wbmm_ros_conversions.cpp)
  target_link_libraries(test_wbmm_ros_conversions ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3 pinocchio wbmm_core geometry_msgs ocs2_msgs std_msgs)
ament_package()
