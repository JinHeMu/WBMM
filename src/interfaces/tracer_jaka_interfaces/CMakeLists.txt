cmake_minimum_required(VERSION 3.8)
project(tracer_jaka_interfaces)

find_package(ament_cmake REQUIRED)

# Add .msg/.srv/.action files later and switch to rosidl_generate_interfaces.
install(DIRECTORY msg srv action
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
