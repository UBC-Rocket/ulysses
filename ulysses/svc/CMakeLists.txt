cmake_minimum_required(VERSION 3.22)

add_library(ulysses_svc OBJECT)

target_link_libraries(ulysses_svc
    PRIVATE
        ulysses_layer_interface
        ulysses_lib
)

target_sources(ulysses_svc
    PRIVATE
        dev/baro/baro.c
        dev/baro/bmp585/baro_bmp585.c
        dev/gnss/gnss.c
        dev/imu/imu.c
        dev/imu/icm40609d/imu_icm40609d.c
        dev/mag/mag.c
        dev/mag/mmc5983/mag_mmc5983.c
        dev/status_led/status_led.c
        dev/status_led/gpio/status_led_gpio.c
)

target_include_directories(ulysses_svc
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
