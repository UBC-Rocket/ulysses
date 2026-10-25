cmake_minimum_required(VERSION 3.22)

add_library(ulysses_dev OBJECT)

target_link_libraries(ulysses_dev
    PRIVATE
        ulysses_layer_interface
        ulysses_lib
        stm32cubemx
)

target_sources(ulysses_dev
    PRIVATE
        bmp585.c
)
