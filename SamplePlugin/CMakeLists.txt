# ##################################################################################################
# Template for building RobWork dependent projects - the src should contain code for putting into
# libraries - the plugins contain code for multiple RobWorkStudio plugins - the test for testing -
# the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this file for adding additional dependencies
# to external projects
# ##################################################################################################

#
# Test CMake version
#
cmake_minimum_required(VERSION 3.10)

# The name of the project.
project(RoViPlugin)

# Set build type to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
message("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)

# Use OpenCV 4
find_package(OpenCV 4 REQUIRED)

# If we want to use ui files add them here
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Need to add the current binary dir to the include directory because the generated source files are
# placed here
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# The shared library to build:
add_library(${PROJECT_NAME} MODULE src/SamplePlugin.cpp src/resources.qrc)
target_link_libraries(
    ${PROJECT_NAME} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${OpenCV_LIBS}
)
