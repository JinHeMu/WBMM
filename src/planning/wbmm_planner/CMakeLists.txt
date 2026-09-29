cmake_minimum_required(VERSION 3.10)
project(wbmm_planner)

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
find_package(wbmm_search REQUIRED)
find_package(wbmm_traj_opt REQUIRED)

# ROS-free orchestration. The node that owns ROS lives in wbmm_planner_ros.
add_library(${PROJECT_NAME} src/whole_body_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  wbmm_core::wbmm_core
  wbmm_search::wbmm_search
  wbmm_traj_opt::wbmm_traj_opt)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(wbmm_pinocchio REQUIRED)
  add_executable(planner_benchmark test/planner_benchmark.cpp)
  target_link_libraries(planner_benchmark ${PROJECT_NAME} wbmm_pinocchio::wbmm_pinocchio)
  target_compile_definitions(planner_benchmark PRIVATE
    WBMM_BENCHMARK_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../../robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf"
    WBMM_BENCHMARK_ESDF="${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/map1/site_remani.npz")

  ament_add_gtest(test_whole_body_planner test/test_whole_body_planner.cpp)
  target_link_libraries(test_whole_body_planner ${PROJECT_NAME})
  target_compile_options(test_whole_body_planner PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 wbmm_core wbmm_search wbmm_traj_opt)
ament_package()
