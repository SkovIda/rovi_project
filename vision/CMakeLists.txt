#
# Test CMake version
#
cmake_minimum_required(VERSION 3.10.0)

# The name of the project.
project(vision)

add_compile_options(-std=c++14)

add_compile_options(-pthread)
# We use the settings that robwork studio uses
#if(NOT DEFINED CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE RelWithDebInfo)
#endif()
#message("-- build type: " ${CMAKE_BUILD_TYPE})

#find_package(RobWork REQUIRED)
#find_package(RobWorkStudio REQUIRED)
find_package(sdurws_robworkstudioapp REQUIRED)
find_package(sdurw_common REQUIRED)
find_package(sdurw_core REQUIRED)
find_package(sdurw_loaders REQUIRED)
find_package(sdurw_models REQUIRED)
find_package(sdurw_kinematics REQUIRED)

add_executable(vision_sparse_stereo rws_sensor_plugin_example.cpp)
target_link_libraries(vision_sparse_stereo ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
