cmake_minimum_required(VERSION 3.10)
project(wbmm_pinocchio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(wbmm_core REQUIRED)
find_package(wbmm_robot_model REQUIRED)

add_library(${PROJECT_NAME}
  src/kinematic_model.cpp
  src/kinematics_data.cpp
  src/frame_kinematics.cpp
  src/sphere_kinematics.cpp
  src/pinocchio_robot_model.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  pinocchio::pinocchio
  wbmm_core::wbmm_core
  wbmm_robot_model::wbmm_robot_model)
ament_target_dependencies(${PROJECT_NAME} PUBLIC Eigen3)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME})
  get_filename_component(WBMM_TEST_URDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../tracer_jaka_description/urdf/tracer_jaka_zu5.urdf"
    ABSOLUTE)
  if(EXISTS "${WBMM_TEST_URDF}")
    target_compile_definitions(test_kinematics PRIVATE
      WBMM_TEST_URDF="${WBMM_TEST_URDF}")
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3 pinocchio wbmm_core wbmm_robot_model)

ament_package()
