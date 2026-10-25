cmake_minimum_required(VERSION 3.22)

add_library(ulysses_svc OBJECT)

target_link_libraries(ulysses_svc
    PRIVATE
        ulysses_layer_interface
        ulysses_lib
)

target_sources(ulysses_svc
    PRIVATE
        dev/mag/mag.c
)
