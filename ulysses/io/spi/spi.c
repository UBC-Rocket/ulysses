#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "ulysses/io/spi.h"

#define MAX_DEVICES (8)

struct io_spi_device {
    SPI_HandleTypeDef *hw_handle;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
};

static struct io_spi_device devices[MAX_DEVICES];
