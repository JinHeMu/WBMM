cmake_minimum_required(VERSION 3.8)
project(wbmm_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(wbmm_core INTERFACE)
target_include_directories(wbmm_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wbmm_core INTERFACE Eigen3::Eigen)
target_compile_features(wbmm_core INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS wbmm_core EXPORT export_wbmm_core)
ament_export_targets(export_wbmm_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wbmm_core test/test_wbmm_core.cpp)
  target_link_libraries(test_wbmm_core wbmm_core)
endif()

ament_package()
