cmake_minimum_required(VERSION 3.8)
project(jaka_sdk_vendor)

find_package(ament_cmake REQUIRED)

# The proprietary SDK is kept in one package.  Consumers receive only the
# include path and absolute shared-library path through the package extras.
install(DIRECTORY include/ DESTINATION include)
install(FILES lib/libjakaAPI.so DESTINATION lib)

ament_package(CONFIG_EXTRAS "cmake/jaka_sdk_vendor-extras.cmake")
