cmake_minimum_required(VERSION 3.10)
project(wbmm_environment)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBZIP REQUIRED libzip)

add_library(${PROJECT_NAME} src/esdf_grid.cpp src/npz_esdf_loader.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PRIVATE ${LIBZIP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${LIBZIP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_environment_skeleton test/test_environment_skeleton.cpp)
  target_link_libraries(test_environment_skeleton ${PROJECT_NAME})
  target_compile_definitions(test_environment_skeleton PRIVATE
    WBMM_ENV_TEST_NPZ="${CMAKE_CURRENT_SOURCE_DIR}/test/resources/test_esdf.npz"
    WBMM_ENV_TEST_NPZ_NO_OBSERVED="${CMAKE_CURRENT_SOURCE_DIR}/test/resources/test_esdf_no_observed.npz")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
