cmake_minimum_required(VERSION 3.8)
project(wbmm_math VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wbmm_core REQUIRED)

add_library(${PROJECT_NAME}
  src/conversions.cpp
  src/linear_algebra.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    wbmm_core::wbmm_core
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME})

  ament_add_gtest(test_joint_mapping test/test_joint_mapping.cpp)
  target_link_libraries(test_joint_mapping ${PROJECT_NAME})

  ament_add_gtest(test_linear_algebra test/test_linear_algebra.cpp)
  target_link_libraries(test_linear_algebra ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 wbmm_core)
ament_export_libraries(${PROJECT_NAME})

ament_package()
