cmake_minimum_required(VERSION 3.8)
project(mm_config)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plan_env REQUIRED)
find_package(urdf REQUIRED)

add_library(mm_config SHARED
  src/mm_config.cpp
)

target_include_directories(mm_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Get plan_env's include directory from its cmake prefix
get_target_property(PLAN_ENV_INCLUDES plan_env::plan_env INTERFACE_INCLUDE_DIRECTORIES)

target_include_directories(mm_config PUBLIC ${PLAN_ENV_INCLUDES})

ament_target_dependencies(mm_config PUBLIC
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
  ament_index_cpp
  urdf
)

target_link_libraries(mm_config PUBLIC plan_env::plan_env)

install(TARGETS mm_config
  EXPORT export_mm_config
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_export_targets(export_mm_config HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  visualization_msgs
  Eigen3
  ament_index_cpp
  plan_env
  urdf
)

ament_package()
