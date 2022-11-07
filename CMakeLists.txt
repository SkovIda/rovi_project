#
# Test CMake version
#
cmake_minimum_required(VERSION 3.10.0)

# The name of the project.
project(graspAnalysis)

add_compile_options(-std=c++14)

# We use the settings that robwork studio uses
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE relwithdebinfo)
endif()

find_package(RobWork REQUIRED)

add_executable(ReachabilityAnalysis reachabilityAnalysis.cpp)
target_link_libraries(ReachabilityAnalysis ${ROBWORK_LIBRARIES})
