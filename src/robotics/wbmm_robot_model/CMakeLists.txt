cmake_minimum_required(VERSION 3.10)
project(wbmm_robot_model)

# Planning/ESDF/FK are latency-sensitive; preserve an explicit caller build type.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wbmm_core REQUIRED)

# urdfdom: the single URDF parser for the whole workspace. It is a plain C++
# library (no ROS client library), so this package stays ROS-free and can be
# used by OCS2 and the ROS-free algorithm packages alike.
find_package(urdfdom REQUIRED)
# yaml-cpp: ROS-free config configuration loader.
find_package(yaml-cpp REQUIRED)

# SHARED so the PRIVATE urdfdom/yaml-cpp imported targets do not leak into the
# exported interface of this library (static libraries would carry a
# $<LINK_ONLY:urdfdom::urdfdom_model> requirement that every consumer would then
# have to find_package again).
add_library(${PROJECT_NAME} SHARED
  src/urdf_description.cpp
  src/robot_model_config.cpp
  src/robot_model_config_loader.cpp
  src/collision_sphere_model.cpp
  src/robot_model_builder.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  wbmm_core::wbmm_core)
# urdfdom and yaml-cpp are implementation details: no public header includes
# them, so they stay PRIVATE.
target_link_libraries(${PROJECT_NAME} PRIVATE
  urdfdom::urdfdom_model
  yaml-cpp)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_robot_description test/test_robot_description.cpp)
  target_link_libraries(test_robot_description ${PROJECT_NAME})

  ament_add_gtest(test_robot_model_config test/test_robot_model_config.cpp)
  target_link_libraries(test_robot_model_config ${PROJECT_NAME})

  ament_add_gtest(test_collision_sphere_model test/test_collision_sphere_model.cpp)
  target_link_libraries(test_collision_sphere_model ${PROJECT_NAME})
  # Lock the real robot's sphere split when the description package is present.
  get_filename_component(WBMM_TEST_URDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../tracer_jaka_description/urdf/tracer_jaka_zu5.urdf"
    ABSOLUTE)
  get_filename_component(WBMM_TEST_CONFIG
    "${CMAKE_CURRENT_SOURCE_DIR}/../tracer_jaka_description/config/wbmm_robot_model.yaml"
    ABSOLUTE)
  if(EXISTS "${WBMM_TEST_URDF}")
    target_compile_definitions(test_collision_sphere_model PRIVATE
      WBMM_TEST_URDF="${WBMM_TEST_URDF}")
    target_compile_definitions(test_robot_description PRIVATE
      WBMM_TEST_URDF="${WBMM_TEST_URDF}")
    target_compile_definitions(test_robot_model_config PRIVATE
      WBMM_TEST_URDF="${WBMM_TEST_URDF}")
  endif()
  if(EXISTS "${WBMM_TEST_CONFIG}")
    target_compile_definitions(test_robot_model_config PRIVATE
      WBMM_TEST_CONFIG="${WBMM_TEST_CONFIG}")
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 wbmm_core)
ament_package()
