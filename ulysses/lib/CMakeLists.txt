cmake_minimum_required(VERSION 3.22)

add_library(ulysses_lib STATIC)

target_link_libraries(ulysses_lib
    PRIVATE
        ulysses_layer_interface
)

target_sources(ulysses_lib
    PRIVATE
        assert/default_handler.c
        bdshot/frame.c
        checksum/crc16.c
)

if(ULYSSES_BUILD_CONTEXT STREQUAL "CM7")
    add_subdirectory(eskf)
    add_subdirectory(flight_controller)
endif()
