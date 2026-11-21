#ifndef ULYSSES_IO_BDSHOT_STM32_BDSHOT
#define ULYSSES_IO_BDSHOT_STM32_BDSHOT

#include <assert.h>
#include <stdint.h>

#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "ulysses/io/bdshot.h"

#define IO_BDSHOT_STM32_TX_BUFFER_SIZE (24)
#define IO_BDSHOT_STM32_RX_BUFFER_SIZE (24)

#define IO_BDSHOT_STM32_MAX_CHANNELS (4)

enum io_bdshot_stm32_channel_direction {
    IO_BDSHOT_STM32_CHANNEL_DIRECTION_OUTPUT,
    IO_BDSHOT_STM32_CHANNEL_DIRECTION_INPUT,
};

struct io_bdshot_stm32_channel_config {
    uint32_t timer_channel; //< HAL timer channel
    DMA_HandleTypeDef *dma; //< DMA instance
    GPIO_TypeDef *gpio;     //< GPIO instance for the timer output
    uint32_t gpio_pin;      //< HAL GPIO pin
    uint32_t *tx_buffer;    //< Buffer to store the frame to transmit
    uint32_t *rx_buffer;    //< Buffer to store the received frame
};

struct io_bdshot_stm32_channel_data {
    const io_bdshot_t *bdshot; //< BDShot instance for the current channel
    uint8_t channel_index;     //< Index of the current channel

    bool is_active;      //< Is the channel active to transceiving?
    bool is_frame_dirty; //< Is the transmitted frame updated?

    uint16_t frame;                      //< Frame being transmitted
    struct io_bdshot_response *response; //< Location to store response

    volatile enum io_bdshot_stm32_channel_direction direction; //< Direction of data flow

    uint32_t original_gpio_af; //< Original alternate function for the GPIO pin
};

struct io_bdshot_stm32_config {
    TIM_HandleTypeDef *timer;
    const struct io_bdshot_stm32_channel_config *channels;
    uint8_t channel_count;
};

struct io_bdshot_stm32_data {
    struct io_bdshot_stm32_channel_data *channels;
    uint32_t active_dma_sources; // TODO: document what this is for
};

void io_bdshot_stm32_init(io_bdshot_t *bdshot, const struct io_bdshot_stm32_config *config,
                          struct io_bdshot_stm32_data *data);

static_assert((IO_BDSHOT_STM32_TX_BUFFER_SIZE * sizeof(uint32_t)) % 32 == 0,
              "Must be D-Cache aligned");
static_assert((IO_BDSHOT_STM32_RX_BUFFER_SIZE * sizeof(uint32_t)) % 32 == 0,
              "Must be D-Cache aligned");

#endif // ULYSSES_IO_BDSHOT_STM32_BDSHOT
