cmake_minimum_required(VERSION 3.22)

add_library(ulysses_svc OBJECT)

target_link_libraries(ulysses_svc
    PRIVATE
        ulysses_layer_interface
        ulysses_lib
)

target_sources(ulysses_svc
    PRIVATE
        dev/lidar/lidar.c
        dev/lidar/vl53l4cx/lidar_vl53l4cx.c
)

# Private headers shared between svc sources
target_include_directories(ulysses_svc
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
