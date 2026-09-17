cmake_minimum_required(VERSION 3.10)
project(wbmm_search)

find_package(ament_cmake REQUIRED)
find_package(wbmm_core REQUIRED)

add_library(${PROJECT_NAME} src/kino_astar.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PUBLIC wbmm_core::wbmm_core)

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
  ament_add_gtest(test_kino_astar test/test_kino_astar.cpp)
  target_link_libraries(test_kino_astar ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(wbmm_core)
ament_package()
