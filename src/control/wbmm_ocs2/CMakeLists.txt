cmake_minimum_required(VERSION 3.14)
project(wbmm_ocs2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_self_collision REQUIRED)
# ocs2_self_collision transitively brings ocs2_pinocchio_interface, whose
# CONFIG_EXTRAS exports OCS2_PINOCCHIO_FLAGS used below.

add_library(${PROJECT_NAME}
  src/FactoryFunctions.cpp
  src/PinocchioMapping.cpp
  src/PreComputation.cpp
  src/Dynamics.cpp
  src/WbmmInterface.cpp
  src/cost/WholeBodyTrajectoryCost.cpp
  src/constraint/EndEffectorConstraint.cpp
  src/constraint/BodyRelativeConstraint.cpp
  src/constraint/EnvironmentCollisionConstraint.cpp
  src/collision/EnvironmentGeometryInterface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${ocs2_ddp_TARGETS}
  ${ocs2_sqp_TARGETS}
  ${ocs2_self_collision_TARGETS}
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  ament_add_gtest(test_wbmm_ocs2 test/test_wbmm_ocs2.cpp)
  target_link_libraries(test_wbmm_ocs2 ${PROJECT_NAME})
  ament_target_dependencies(test_wbmm_ocs2 ament_index_cpp)
  target_compile_definitions(test_wbmm_ocs2 PRIVATE
    WBMM_OCS2_TEST_URDF_FALLBACK="${CMAKE_CURRENT_SOURCE_DIR}/../../robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf"
    WBMM_OCS2_TEST_TASK_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/resources/task.info")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ocs2_ddp ocs2_sqp ocs2_self_collision)
ament_package()
