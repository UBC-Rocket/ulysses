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

# There are no dynamic linkers for our embedded system. We must
# statically link all libraries.
set(BUILD_SHARED_LIBS OFF)

# Set the project name
set(CMAKE_PROJECT_NAME ulysses)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})

option(ULYSSES_ENABLE_DEBUG_LOGGING "Enable serial debug logging" ON)
option(ULYSSES_USE_SD_CARD "Enable SD card initialization (requires SD card to be inserted)" OFF)

# Display a summary of the build configuration
message("Build type: " ${CMAKE_BUILD_TYPE})
message("Debug logging: " ${ULYSSES_ENABLE_DEBUG_LOGGING})
message("SD card initialization: " ${ULYSSES_USE_SD_CARD})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add dependencies
include(cmake/printf-library.cmake)

FetchContent_MakeAvailable(printf)

# Library requires a native 64-bit integer type, so must be be compiled
# with at least C99 standard
set_target_properties(printf PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

set(DEBUG_MODULE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/debug/log.c
)

set(RTOS_TASKS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/controls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/mission_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/state_estimation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tasks/debug_logging.c
)

set(SPI_DRIVER_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/SPI_queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/SPI_device_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/bmi088_accel_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/bmi088_gyro_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/ms5611_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/BMI088_accel.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/BMI088_gyro.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi_drivers/MS5611_baro.c
)

set(ALGO_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/algorithms/ekf.c
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
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/state_exchange.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/state_estimation/state_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/interrupts.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/SD_logging/log_writer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/SD_logging/log_service.c
    ${DEBUG_MODULE_SRC}
    ${ALGO_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/spi_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/algorithms
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    BMI_ACC_Chip_Select_GPIO_Port=BMI_ACC_CS_GPIO_Port
    BMI_ACC_Chip_Select_Pin=BMI_ACC_CS_Pin
    BMI_GYRO_Chip_Select_GPIO_Port=BMI_GYRO_CS_GPIO_Port
    BMI_GYRO_Chip_Select_Pin=BMI_GYRO_CS_Pin
    $<$<BOOL:${ULYSSES_USE_SD_CARD}>:ULYSSES_USE_SD_CARD>
    $<$<BOOL:${ULYSSES_ENABLE_DEBUG_LOGGING}>:ULYSSES_ENABLE_DEBUG_LOGGING>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    printf::printf
)
