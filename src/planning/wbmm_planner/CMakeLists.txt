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
find_package(wbmm_environment REQUIRED)
find_package(wbmm_collision REQUIRED)
find_package(wbmm_robot_model REQUIRED)

# Search, trajectory optimization and orchestration form one ROS-free library.
add_library(${PROJECT_NAME}
  src/search/kino_astar.cpp
  src/search/arm_seed_search.cpp
  src/search/whole_body_rrt.cpp
  src/optimization/whole_body_trajectory_builder.cpp
  src/optimization/whole_body_optimizer.cpp
  src/whole_body_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  wbmm_core::wbmm_core
  wbmm_environment::wbmm_environment
  wbmm_collision::wbmm_collision
  wbmm_robot_model::wbmm_robot_model)

add_executable(kino_astar_demo examples/kino_astar_demo.cpp)
target_link_libraries(kino_astar_demo ${PROJECT_NAME})
target_compile_options(kino_astar_demo PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS kino_astar_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/plot_kino_astar.py DESTINATION lib/${PROJECT_NAME})

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

  foreach(test_name IN ITEMS test_kino_astar test_arm_seed_search test_whole_body_rrt)
    ament_add_gtest(${test_name} test/search/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
  ament_add_gtest(test_search_collision_integration
    test/search/test_search_collision_integration.cpp)
  target_link_libraries(test_search_collision_integration ${PROJECT_NAME})

  foreach(test_name IN ITEMS test_minco test_whole_body_trajectory_builder test_whole_body_optimizer)
    ament_add_gtest(${test_name} test/optimization/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
  get_filename_component(WBMM_TEST_URDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../../robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf"
    ABSOLUTE)
  get_filename_component(WBMM_TEST_ESDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/map1/site_remani.npz" ABSOLUTE)
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
ament_export_dependencies(Eigen3 wbmm_core wbmm_environment wbmm_collision wbmm_robot_model)
ament_package()
