cmake_minimum_required(VERSION 3.22)

option(ULYSSES_BUILD_NATIVE "Build for the current host" OFF)
option(ULYSSES_BUILD_TESTS "Build unit tests" OFF)

# Interface library to link against for accessing public headers
add_library(ulysses_public_headers INTERFACE)

target_include_directories(ulysses_public_headers
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

# Interface library to link against for generic layer build requirements
add_library(ulysses_layer_interface INTERFACE)

target_link_libraries(ulysses_layer_interface
    INTERFACE
        ulysses_public_headers
)

target_compile_options(ulysses_layer_interface
    INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
)

target_compile_features(ulysses_layer_interface
    INTERFACE
        c_std_11
)

add_subdirectory(lib)

if(NOT ULYSSES_BUILD_NATIVE)
    # ST's VL53L4CX lidar driver headers, from CubeMX's X-CUBE-TOF1. SYSTEM so the layer warning
    # flags don't apply to them.
    set(VL53L4CX_DIR ${CMAKE_CURRENT_LIST_DIR}/cfp/Drivers/BSP/Components/vl53l4cx)
    add_library(vl53l4cx_headers INTERFACE)
    target_include_directories(vl53l4cx_headers
        SYSTEM INTERFACE
            ${VL53L4CX_DIR}
            ${VL53L4CX_DIR}/modules
            ${VL53L4CX_DIR}/porting
    )

    # board.h exposes the lidar's ST bus functions
    target_link_libraries(ulysses_public_headers INTERFACE vl53l4cx_headers)

    add_subdirectory(io)
    add_subdirectory(svc)
endif()

if(ULYSSES_BUILD_TESTS)
    add_subdirectory(tests)
endif()
