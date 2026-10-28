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
        i2c/i2c_dev.c
        i2c/i2c_bus.c
        i2c/stm32/i2c_stm32.c
        uart/uart.c
        uart/stm32/uart_stm32.c
)

target_include_directories(ulysses_io
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Board wiring, one file per core
if(ULYSSES_BUILD_CONTEXT STREQUAL "CM7")
    target_sources(ulysses_io PRIVATE board/board_cm7.c)
elseif(ULYSSES_BUILD_CONTEXT STREQUAL "CM4")
    target_sources(ulysses_io PRIVATE board/board_cm4.c)
endif()
