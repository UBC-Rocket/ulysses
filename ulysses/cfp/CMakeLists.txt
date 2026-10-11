cmake_minimum_required(VERSION 3.22)
include(ExternalProject)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Set the project name
set(CMAKE_PROJECT_NAME ulysses)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Include external project file
#
# We are using our own version of the file, since the one generated
# by STM32CubeMX has poorly named binaries, and unecessarily creates
# a build folder for each core under a path different from PROJECT_BINARY_DIR.
include("mx-generated-user.cmake")
