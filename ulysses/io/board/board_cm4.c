// CubeMX handles
#include "i2c.h"

#include "i2c/i2c_bus.h"
#include "i2c/i2c_dev.h"
#include "i2c/stm32/i2c_stm32.h"

#include "ulysses/io/board.h"

// TODO: placeholder wiring, fill in once the board layout is final

// I2C bus and 7-bit address
#define FUEL_GAUGE_BUS (&i2c3_bus)
#define FUEL_GAUGE_ADDR (0x55U)
#define BMS_BUS (&i2c4_bus)
#define BMS_ADDR (0x08U)

// Devices below have no IO driver yet. Uncomment when their driver lands.

// SPI handle and chip-select pin
// #define RADIO_SPI (&hspi5)
// #define RADIO_CS_PORT (RADIO_NSS_GPIO_Port)
// #define RADIO_CS_PIN (RADIO_NSS_Pin)
// TODO: FLASH shares SPI5 with the radio but has no chip-select pin yet
// #define FLASH_SPI (&hspi5)
// #define FLASH_CS_PORT
// #define FLASH_CS_PIN
// #define GATE_DRIVER1_SPI (&hspi6)
// #define GATE_DRIVER1_CS_PORT (GATE_DRIVER1_CS_GPIO_Port)
// #define GATE_DRIVER1_CS_PIN (GATE_DRIVER1_CS_Pin)
// #define GATE_DRIVER2_SPI (&hspi6)
// #define GATE_DRIVER2_CS_PORT (GATE_DRIVER2_CS_GPIO_Port)
// #define GATE_DRIVER2_CS_PIN (GATE_DRIVER2_CS_Pin)

// UART handle
// #define DEBUG_UART (&hlpuart1)
// #define ESC_MCU1_UART (&huart6)
// #define ESC_MCU2_UART (&huart8)
// #define RFD900X_UART (&huart1)
// #define RTK_GNSS_UART (&huart3)
// #define RTK_CORRECTION_UART (&huart2)

// SD handle
// #define SDCARD_SD (&hsd2)

// GPIO port and pin
// #define RADIO_NRST_PORT (RADIO_NRST_GPIO_Port)
// #define RADIO_NRST_PIN (RADIO_NRST_Pin)
// #define RADIO_BUSY_PORT (RADIO_BUSY_GPIO_Port)
// #define RADIO_BUSY_PIN (RADIO_BUSY_Pin)
// #define GATE_DRIVER1_NFAULT_PORT (GATE_DRIVER1_nFAULT_GPIO_Port)
// #define GATE_DRIVER1_NFAULT_PIN (GATE_DRIVER1_nFAULT_Pin)
// #define GATE_DRIVER2_NFAULT_PORT (GATE_DRIVER2_nFAULT_GPIO_Port)
// #define GATE_DRIVER2_NFAULT_PIN (GATE_DRIVER2_nFAULT_Pin)
// #define RTK_GNSS_NRST_PORT (RTK_GNSS_NRST_GPIO_Port)
// #define RTK_GNSS_NRST_PIN (RTK_GNSS_NRST_Pin)
// #define RTK_GNSS_INT_PORT (RTK_GNSS_INT_GPIO_Port)
// #define RTK_GNSS_INT_PIN (RTK_GNSS_INT_Pin)
// #define STAT_LED_RED_PORT (STAT_LEDR_GPIO_Port)
// #define STAT_LED_RED_PIN (STAT_LEDR_Pin)
// #define STAT_LED_GREEN_PORT (STAT_LEDG_GPIO_Port)
// #define STAT_LED_GREEN_PIN (STAT_LEDG_Pin)
// #define STAT_LED_ORANGE_PORT (STAT_LEDO_GPIO_Port)
// #define STAT_LED_ORANGE_PIN (STAT_LEDO_Pin)

static struct io_i2c_bus i2c3_bus;
static struct io_i2c_bus i2c4_bus;
static const struct io_i2c_stm32_config i2c3_config = {.hw_handle = &hi2c3};
static const struct io_i2c_stm32_config i2c4_config = {.hw_handle = &hi2c4};

const io_i2c_dev_t IO_FUEL_GAUGE = {.bus = FUEL_GAUGE_BUS, .addr = FUEL_GAUGE_ADDR};
const io_i2c_dev_t IO_BMS = {.bus = BMS_BUS, .addr = BMS_ADDR};

void io_board_init(void)
{
    io_i2c_stm32_init(&i2c3_bus, &i2c3_config);
    io_i2c_stm32_init(&i2c4_bus, &i2c4_config);
}
