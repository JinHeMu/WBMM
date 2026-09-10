cmake_minimum_required(VERSION 3.8)
project(wbmm_core VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

# Offline mode is used when no ROS 2 / ament environment is available.
# It keeps wbmm_core a normal CMake library and uses CTest + GTest.
# Accepted spellings for the same offline switch.
option(WBMM_BUILD_OFFLINE "Build wbmm_core without ament/ROS (plain CMake)" OFF)
option(WBMM_OFFLINE "Short alias for WBMM_BUILD_OFFLINE" OFF)
option(WBMM_ENABLE_AMENT "Build with ament/ROS integration" ON)
set(WBMM_USE_AMENT ON)
if(WBMM_BUILD_OFFLINE OR WBMM_OFFLINE OR NOT WBMM_ENABLE_AMENT)
  set(WBMM_USE_AMENT OFF)
  include(CTest)
else()
  find_package(ament_cmake REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  src/status.cpp
  src/validation.cpp
  src/math/conversions.cpp
  src/math/linear_algebra.cpp
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

function(wbmm_core_add_gtest target source)
  if(WBMM_USE_AMENT)
    ament_add_gtest(${target} ${source})
    target_link_libraries(${target} ${PROJECT_NAME})
  else()
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE ${PROJECT_NAME} GTest::gtest_main)
    add_test(NAME ${target} COMMAND ${target})
  endif()
endfunction()

if(BUILD_TESTING)
  if(WBMM_USE_AMENT)
    find_package(ament_cmake_gtest REQUIRED)
  else()
    enable_testing()
    find_package(GTest REQUIRED)
  endif()

  wbmm_core_add_gtest(test_status test/test_status.cpp)
  wbmm_core_add_gtest(test_validation test/test_validation.cpp)
  wbmm_core_add_gtest(test_ports test/test_ports.cpp)
  wbmm_core_add_gtest(test_math_conversions test/test_math_conversions.cpp)
  wbmm_core_add_gtest(test_math_joint_mapping test/test_math_joint_mapping.cpp)
  wbmm_core_add_gtest(test_math_linear_algebra test/test_math_linear_algebra.cpp)
endif()

if(WBMM_USE_AMENT)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_export_dependencies(Eigen3)
  ament_export_libraries(${PROJECT_NAME})

  ament_package()
endif()
