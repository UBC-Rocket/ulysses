cmake_minimum_required(VERSION 3.22)
include(ExternalProject)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

option(ULYSSES_BUILD_NATIVE "Build for the host computer" OFF)
option(ULYSSES_BUILD_TESTS "Build unit tests" OFF)

# Set the project name
set(CMAKE_PROJECT_NAME ulysses)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)

if(ULYSSES_BUILD_TESTS)
    enable_testing()
endif()

if(ULYSSES_BUILD_NATIVE)
    set(ULYSSES_BUILD_CONTEXT "NATIVE")
    set(ULYSSES_ROOT ${CMAKE_CURRENT_LIST_DIR}/../)

    add_subdirectory(${ULYSSES_ROOT} ${PROJECT_BINARY_DIR}/ulysses)
else()
    # Include external project file
    #
    # We are using our own version of the file, since the one generated
    # by STM32CubeMX has poorly named binaries, and unecessarily creates
    # a build folder for each core under a path different from PROJECT_BINARY_DIR.
    include("mx-generated-user.cmake")
endif()
