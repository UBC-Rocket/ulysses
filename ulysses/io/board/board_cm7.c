// CubeMX handles
#include "i2c.h"

#include "i2c/i2c_bus.h"
#include "i2c/i2c_dev.h"
#include "i2c/stm32/i2c_stm32.h"

#include "ulysses/io/board.h"

// TODO: placeholder wiring, fill in once the board layout is final

// I2C bus and 7-bit address
#define BARO_BUS (&i2c1_bus)
#define BARO_ADDR (0x76U)
#define LIDAR_BUS (&i2c2_bus)
#define LIDAR_ADDR (0x62U)

// Devices below have no IO driver yet. Uncomment when their driver lands.

// SPI handle and chip-select pin
// #define IMU1_SPI (&hspi1)
// #define IMU1_CS_PORT (IMU1_NSS_GPIO_Port)
// #define IMU1_CS_PIN (IMU1_NSS_Pin)
// #define IMU2_SPI (&hspi2)
// #define IMU2_CS_PORT (IMU2_NSS_GPIO_Port)
// #define IMU2_CS_PIN (IMU2_NSS_Pin)
// #define IMU3_SPI (&hspi3)
// #define IMU3_CS_PORT (IMU3_NSS_GPIO_Port)
// #define IMU3_CS_PIN (IMU3_NSS_Pin)
// #define MAG_SPI (&hspi4)
// #define MAG_CS_PORT (MAG_NSS_GPIO_Port)
// #define MAG_CS_PIN (MAG_NSS_Pin)

// UART handle
// #define MAUWB1_UART (&huart4)
// #define MAUWB2_UART (&huart5)
// #define TTL_UART (&huart7)

// PWM timer and channel
// #define ESC1_TIM (&htim1)
// #define ESC1_CHANNEL (TIM_CHANNEL_1)
// #define ESC2_TIM (&htim1)
// #define ESC2_CHANNEL (TIM_CHANNEL_2)
// #define LIN_ACTUATOR1_TIM (&htim2)
// #define LIN_ACTUATOR1_CHANNEL (TIM_CHANNEL_1)
// #define LIN_ACTUATOR2_TIM (&htim2)
// #define LIN_ACTUATOR2_CHANNEL (TIM_CHANNEL_2)

// FDCAN handle
// #define LIN_ACTUATOR_FDCAN (&hfdcan1)

// GPIO port and pin
// #define BARO_INT_PORT (BARO_INT_GPIO_Port)
// #define BARO_INT_PIN (BARO_INT_Pin)
// #define LIDAR_EN_PORT (LiDAR_EN_GPIO_Port)
// #define LIDAR_EN_PIN (LiDAR_EN_Pin)
// #define LIDAR_INT_PORT (LiDAR_INT_GPIO_Port)
// #define LIDAR_INT_PIN (LiDAR_INT_Pin)
// #define IMU1_INT1_PORT (IMU1_INT1_GPIO_Port)
// #define IMU1_INT1_PIN (IMU1_INT1_Pin)
// #define IMU1_INT2_PORT (IMU1_INT2_GPIO_Port)
// #define IMU1_INT2_PIN (IMU1_INT2_Pin)
// #define IMU2_INT1_PORT (IMU2_INT1_GPIO_Port)
// #define IMU2_INT1_PIN (IMU2_INT1_Pin)
// #define IMU2_INT2_PORT (IMU2_INT2_GPIO_Port)
// #define IMU2_INT2_PIN (IMU2_INT2_Pin)
// #define IMU3_INT1_PORT (IMU3_INT1_GPIO_Port)
// #define IMU3_INT1_PIN (IMU3_INT1_Pin)
// #define IMU3_INT2_PORT (IMU3_INT2_GPIO_Port)
// #define IMU3_INT2_PIN (IMU3_INT2_Pin)
// #define MAG_INT_PORT (MAG_INT_GPIO_Port)
// #define MAG_INT_PIN (MAG_INT_Pin)
// #define TTL_TX_EN_PORT (TTL_TX_EN_GPIO_Port)
// #define TTL_TX_EN_PIN (TTL_TX_EN_Pin)
// #define TTL_LVL_SHIFT_EN_PORT (TTL_LVL_SHIFT_EN_GPIO_Port)
// #define TTL_LVL_SHIFT_EN_PIN (TTL_LVL_SHIFT_EN_Pin)
// #define GATE_DRIVER1_EN_PORT (GATE_DRIVER1_EN_GPIO_Port)
// #define GATE_DRIVER1_EN_PIN (GATE_DRIVER1_EN_Pin)
// #define GATE_DRIVER2_EN_PORT (GATE_DRIVER2_EN_GPIO_Port)
// #define GATE_DRIVER2_EN_PIN (GATE_DRIVER2_EN_Pin)

static struct io_i2c_bus i2c1_bus;
static struct io_i2c_bus i2c2_bus;
static const struct io_i2c_stm32_config i2c1_config = {.hw_handle = &hi2c1};
static const struct io_i2c_stm32_config i2c2_config = {.hw_handle = &hi2c2};

const io_i2c_dev_t IO_BARO = {.bus = BARO_BUS, .addr = BARO_ADDR};
const io_i2c_dev_t IO_LIDAR = {.bus = LIDAR_BUS, .addr = LIDAR_ADDR};

void io_board_init(void)
{
    io_i2c_stm32_init(&i2c1_bus, &i2c1_config);
    io_i2c_stm32_init(&i2c2_bus, &i2c2_config);
}
