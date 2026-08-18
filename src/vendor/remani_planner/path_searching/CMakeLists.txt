cmake_minimum_required(VERSION 3.8)
project(path_searching)

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
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(ompl REQUIRED)
find_package(plan_env REQUIRED)
find_package(mm_config REQUIRED)
find_package(traj_utils REQUIRED)

add_library(path_searching SHARED
  src/kino_astar.cpp
  src/sample_mani_RRT.cpp
  src/rrt.cpp
)

target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

ament_target_dependencies(path_searching PUBLIC
  rclcpp
  std_msgs
  visualization_msgs
)

target_link_libraries(path_searching PUBLIC
  ${PCL_LIBRARIES}
  ${OMPL_LIBRARIES}
  plan_env::plan_env
  mm_config::mm_config
  traj_utils::traj_utils_core
)

ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  visualization_msgs
  Eigen3
  PCL
  ompl
  plan_env
  mm_config
  traj_utils
)

install(TARGETS path_searching
  EXPORT export_path_searching
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
