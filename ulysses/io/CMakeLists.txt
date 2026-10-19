cmake_minimum_required(VERSION 3.22)

add_library(ulysses_io OBJECT)

target_link_libraries(ulysses_io
    PRIVATE
        ulysses_layer_interface
        ulysses_lib
        stm32cubemx
)

target_include_directories(ulysses_io
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_sources(ulysses_io
    PRIVATE
        assert/handler.c
        irq/irq.c
        irq/isr_hooks.c
        spi/spi.c
        spi/stm32/spi_stm32.c
)
