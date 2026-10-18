cmake_minimum_required(VERSION 3.22)

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
add_subdirectory(io)
