cmake_minimum_required(VERSION 3.10)
project(wbmm_traj_opt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wbmm_core REQUIRED)
find_package(wbmm_environment REQUIRED)
find_package(wbmm_collision REQUIRED)

# ROS-free by design: the MINCO representation, the banded KKT solver, the
# vendored L-BFGS and the trajectory builder depend only on Eigen and the
# wbmm_core contracts. The ROS node that drives this lands in a separate
# package.
add_library(${PROJECT_NAME}
  src/whole_body_trajectory_builder.cpp
  src/whole_body_optimizer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  wbmm_core::wbmm_core
  wbmm_environment::wbmm_environment
  wbmm_collision::wbmm_collision)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_minco test/test_minco.cpp)
  target_link_libraries(test_minco ${PROJECT_NAME})
  target_compile_options(test_minco PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_whole_body_trajectory_builder
    test/test_whole_body_trajectory_builder.cpp)
  target_link_libraries(test_whole_body_trajectory_builder ${PROJECT_NAME})
  target_compile_options(test_whole_body_trajectory_builder
    PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_whole_body_optimizer test/test_whole_body_optimizer.cpp)
  target_link_libraries(test_whole_body_optimizer ${PROJECT_NAME})
  target_compile_options(test_whole_body_optimizer PRIVATE -Wall -Wextra -Wpedantic)
  get_filename_component(WBMM_TEST_URDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf"
    ABSOLUTE)
  get_filename_component(WBMM_TEST_ESDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../maps/map1/site_remani.npz" ABSOLUTE)
  if(EXISTS "${WBMM_TEST_URDF}")
    target_compile_definitions(test_whole_body_optimizer PRIVATE
      WBMM_TEST_URDF="${WBMM_TEST_URDF}")
  endif()
  if(EXISTS "${WBMM_TEST_ESDF}")
    target_compile_definitions(test_whole_body_optimizer PRIVATE
      WBMM_TEST_ESDF="${WBMM_TEST_ESDF}")
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 wbmm_core wbmm_environment wbmm_collision)
ament_package()
