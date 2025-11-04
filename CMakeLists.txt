cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ulysses)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

set(RTOS_TASKS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/motor_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/radio_communication.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/sensor_fusion.c
)

set(SPI_DRIVER_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/SPI_queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/SPI_device_interactions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/BMI088_accel.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/BMI088_gyro.c
)


# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${RTOS_TASKS_SRC}
    ${SPI_DRIVER_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/interrupts.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/spi_drivers
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    BMI_ACC_Chip_Select_GPIO_Port=BMI_ACC_CS_GPIO_Port
    BMI_ACC_Chip_Select_Pin=BMI_ACC_CS_Pin
    BMI_GYRO_Chip_Select_GPIO_Port=BMI_GYRO_CS_GPIO_Port
    BMI_GYRO_Chip_Select_Pin=BMI_GYRO_CS_Pin
)

# Add CLI option to disable the SD card peripheral initialization
if(ULYSSES_USE_SD_CARD)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ULYSSES_USE_SD_CARD)
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
