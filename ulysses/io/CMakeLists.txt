cmake_minimum_required(VERSION 3.22)

add_library(ulysses_io OBJECT)

target_link_libraries(ulysses_io
    PRIVATE
        ulysses_layer_interface
        ulysses_lib
        stm32cubemx
)

target_sources(ulysses_io
    PRIVATE
        assert/handler.c
        irq/irq.c
        irq/isr_hooks.c
)
