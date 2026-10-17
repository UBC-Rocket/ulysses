cmake_minimum_required(VERSION 3.22)

add_library(ulysses_lib STATIC)

target_link_libraries(ulysses_lib
    PRIVATE
        ulysses_layer_interface
)

target_sources(ulysses_lib
    PRIVATE
        assert/default_handler.c
)
