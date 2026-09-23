cmake_minimum_required(VERSION 3.10)
project(wbmm_collision)

find_package(ament_cmake REQUIRED)
find_package(wbmm_core REQUIRED)
find_package(wbmm_environment REQUIRED)

add_library(${PROJECT_NAME} src/environment_collision_checker.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PUBLIC
  wbmm_core::wbmm_core wbmm_environment::wbmm_environment)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(wbmm_search REQUIRED)

  ament_add_gtest(test_environment_collision_checker test/test_environment_collision_checker.cpp)
  target_link_libraries(test_environment_collision_checker ${PROJECT_NAME})

  ament_add_gtest(test_search_collision_integration test/test_search_collision_integration.cpp)
  target_link_libraries(test_search_collision_integration
    ${PROJECT_NAME} wbmm_search::wbmm_search)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(wbmm_core wbmm_environment)
ament_package()
