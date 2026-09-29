cmake_minimum_required(VERSION 3.10)
project(wbmm_collision)

# Planning/ESDF/FK are latency-sensitive; preserve an explicit caller build type.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(wbmm_core REQUIRED)
find_package(wbmm_environment REQUIRED)
# Collision geometry is not parsed here any more: the shared wbmm_robot_model
# owns the single URDF read and the shared CollisionSphereModel.
find_package(wbmm_robot_model REQUIRED)
# Sphere FK lives in wbmm_pinocchio; the checker updates one KinematicsData
# per query instead of recomputing full-tree FK per link.
find_package(wbmm_pinocchio REQUIRED)

add_library(${PROJECT_NAME}
  src/esdf_checker.cpp
  src/trajectory_spheres.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PUBLIC
  wbmm_core::wbmm_core
  wbmm_environment::wbmm_environment
  wbmm_robot_model::wbmm_robot_model
  wbmm_pinocchio::wbmm_pinocchio)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_esdf_checker
    test/test_esdf_checker.cpp)
  target_link_libraries(test_esdf_checker ${PROJECT_NAME})

  ament_add_gtest(test_trajectory_spheres
    test/test_trajectory_spheres.cpp)
  target_link_libraries(test_trajectory_spheres ${PROJECT_NAME})
  # Lock the real robot's sphere split when the description package is present.
  get_filename_component(WBMM_TEST_URDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../tracer_jaka_description/urdf/tracer_jaka_zu5.urdf"
    ABSOLUTE)
  if(EXISTS "${WBMM_TEST_URDF}")
    target_compile_definitions(test_trajectory_spheres PRIVATE
      WBMM_TEST_URDF="${WBMM_TEST_URDF}")
  endif()

endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(wbmm_core wbmm_environment wbmm_robot_model wbmm_pinocchio)
ament_package()
