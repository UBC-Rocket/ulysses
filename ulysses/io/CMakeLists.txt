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
        delay/delay.c
        irq/irq.c
        irq/isr_hooks.c
        i2c/i2c_dev.c
        i2c/i2c_bus.c
        i2c/stm32/i2c_stm32.c
)

# Private headers shared between io sources
target_include_directories(ulysses_io
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
