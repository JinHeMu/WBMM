cmake_minimum_required(VERSION 3.10)
project(wbmm_collision)

find_package(ament_cmake REQUIRED)
find_package(wbmm_core REQUIRED)
find_package(wbmm_environment REQUIRED)

# tinyxml2 is a plain system library, not a ROS package: the URDF loader stays
# ROS-free on purpose. urdf::Model would have pulled ROS into this package.
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

add_library(${PROJECT_NAME}
  src/environment_collision_checker.cpp
  src/urdf_collision_model.cpp
  src/whole_body_kinematics.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PRIVATE ${TINYXML2_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PUBLIC
  wbmm_core::wbmm_core wbmm_environment::wbmm_environment)
target_link_libraries(${PROJECT_NAME} PRIVATE ${TINYXML2_LIBRARIES})

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(wbmm_search REQUIRED)

  ament_add_gtest(test_environment_collision_checker test/test_environment_collision_checker.cpp)
  target_link_libraries(test_environment_collision_checker ${PROJECT_NAME})

  ament_add_gtest(test_urdf_collision_model test/test_urdf_collision_model.cpp)
  target_link_libraries(test_urdf_collision_model ${PROJECT_NAME})
  # Lock the real robot's sphere split when the description package is present.
  get_filename_component(WBMM_TEST_URDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../tracer_jaka_description/urdf/tracer_jaka_zu5.urdf"
    ABSOLUTE)
  if(EXISTS "${WBMM_TEST_URDF}")
    target_compile_definitions(test_urdf_collision_model PRIVATE
      WBMM_TEST_URDF="${WBMM_TEST_URDF}")
  endif()

  ament_add_gtest(test_whole_body_kinematics test/test_whole_body_kinematics.cpp)
  target_link_libraries(test_whole_body_kinematics ${PROJECT_NAME})
  target_compile_definitions(test_whole_body_kinematics PRIVATE
    WBMM_TEST_URDF="${WBMM_TEST_URDF}")

  ament_add_gtest(test_search_collision_integration test/test_search_collision_integration.cpp)
  target_link_libraries(test_search_collision_integration
    ${PROJECT_NAME} wbmm_search::wbmm_search)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(wbmm_core wbmm_environment)
ament_package()
