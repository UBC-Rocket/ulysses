#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "stm32h7xx.h" // IWYU pragma: keep
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_tim.h"

#include "ulysses/io/bdshot.h"
#include "ulysses/io/irq.h"
#include "ulysses/io/status.h"
#include "ulysses/lib/assert.h"
#include "ulysses/lib/bdshot/protocol.h"

#include "bdshot/bdshot.h"
#include "bdshot/stm32/bdshot.h"
#include "bdshot/stm32/hal_ll_helpers.h"

/**
 * @brief Number of idle bits at the end of a TX frame
 * 
 * Not necessary, but it makes the communication more reliable
 * by ensuring the ESC receives our frame in full in case the switch
 * from TX to RX is noisy.
 */
#define TX_IDLE_BITS (2)

/**
 * @brief Number of bits in a TX frame
 */
#define TX_FRAME_SIZE (BDSHOT_FRAME_BITS + TX_IDLE_BITS)

/**
 * @brief Number of bits in a RX frame
 *
 * Since this is the number of rising and falling edges,
 * it is at most as the number of bits in the longest wire frame.
 * The longest wire frame is just when the DShot frame is alternating bits.
 */
#define RX_FRAME_SIZE (BDSHOT_TELEMETRY_WIRE_BITS)

/**
 * @brief Timer ticks for DShot bits
 *
 * Currently configured for DShot300.
 *
 * For some version DShot[XXX], the bit rate is XXX kb/s (e.g. DShot300 is 300 kb/s).
 *
 * Bit period in seconds is `1/(bit rate)`. From this, we can calculate the bit ticks
 * by using `(timer peripheral clock in hertz)*(bit period in seconds)`.
 *
 * The high time of a 1 bit is not well documented, but most FCs and ESCs use 75% of the bit
 * period/ticks. By the protocol definition, the high time of a 0 bit is always half that of
 * a 1 bit.
 */
#define T1H_TICKS (600)
#define T0H_TICKS (300)
#define BIT_TICKS (800)

/**
 * @brief Number of ticks for 1 bit of the wire telemetry response
 * 
 * Wire telemetry frame is sent at `(5/4)*(bit rate)`.
 */
#define WIRE_TELEMETRY_BIT_TICKS ((BIT_TICKS * 4.0f) / 5.0f)

static_assert(IO_BDSHOT_STM32_TX_BUFFER_SIZE >= TX_FRAME_SIZE, "TX buffer size is too small");
static_assert(IO_BDSHOT_STM32_RX_BUFFER_SIZE >= RX_FRAME_SIZE, "RX buffer size is too small");

static void frame_build_dma_tx_buffer(uint32_t *buffer, uint16_t frame);
static bool wire_frame_decode_from_signal(uint32_t *frame, const uint32_t *edge_times,
                                          uint32_t edge_count);
static void signal_line_set_to_tx(TIM_HandleTypeDef *timer,
                                  const struct io_bdshot_stm32_channel_config *channel_config);
static void signal_line_set_to_rx(TIM_HandleTypeDef *timer,
                                  const struct io_bdshot_stm32_channel_config *channel_config);
static void on_dma_transfer_complete_isr(void *user_data);

static void frame_build_dma_tx_buffer(uint32_t *buffer, uint16_t frame)
{
    // Pack the frame
    for (uint8_t i = 0; i < BDSHOT_FRAME_BITS; i++) {
        buffer[i] = (frame & 0x8000) != 0 ? T1H_TICKS : T0H_TICKS;
        frame <<= 1;
    }

    // Pack the idle bits
    for (uint8_t i = 0; i < TX_IDLE_BITS; i++) {
        buffer[BDSHOT_FRAME_BITS + i] = 0;
    }
}

static bool wire_frame_decode_from_signal(uint32_t *frame, const uint32_t *edge_times,
                                          uint32_t edge_count)
{
    if (edge_count == 0 || edge_count > RX_FRAME_SIZE) {
        return false;
    }

    bool is_success = true;

    uint8_t bit = 0;
    uint32_t wire_frame = 0;
    uint32_t wire_bits_written = 0;

    for (uint32_t i = 0; i < (edge_count - 1); i++) {
        // Written enough bits for a full frame
        if (wire_bits_written >= BDSHOT_TELEMETRY_WIRE_BITS) {
            break;
        }

        uint32_t width = edge_times[i + 1] - edge_times[i];
        uint32_t bit_count = (uint32_t)roundf((float)width / WIRE_TELEMETRY_BIT_TICKS);

        if (bit_count <= 0 || bit_count > BDSHOT_TELEMETRY_WIRE_BITS ||
            (wire_bits_written + bit_count) > BDSHOT_TELEMETRY_WIRE_BITS) {
            is_success = false;
            break;
        }

        for (uint32_t j = 0; j < bit_count; j++) {
            wire_frame = (wire_frame << 1) | bit;
            wire_bits_written++;
        }

        // Every edge we encounter means the bit value we received is flipped
        bit ^= 1;
    }

    if (is_success) {
        // Bits in the rest of the frame is the same as the logic level following the last edge
        for (uint32_t i = wire_bits_written; i < BDSHOT_TELEMETRY_WIRE_BITS; i++) {
            wire_frame = (wire_frame << 1) | bit;
        }

        *frame = wire_frame;
    }

    return is_success;
}

static void signal_line_set_to_tx(TIM_HandleTypeDef *timer,
                                  const struct io_bdshot_stm32_channel_config *channel_config)
{
    DMA_HandleTypeDef *dma = channel_config->dma;
    const uint32_t hal_tim_channel = channel_config->timer_channel;
    const uint32_t ll_tim_channel = hal_to_ll_tim_channel(hal_tim_channel);

    // Disable DMA since since we are re-configuring channel to do transfers
    // from memory to peripheral
    __HAL_DMA_DISABLE(dma);

    TIM_CCxChannelCmd(timer->Instance, hal_tim_channel, TIM_CCx_DISABLE);

    LL_TIM_OC_SetMode(timer->Instance, ll_tim_channel, LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_ConfigOutput(timer->Instance, ll_tim_channel, LL_TIM_OCPOLARITY_HIGH);

    LL_TIM_OC_DisablePreload(timer->Instance, ll_tim_channel);
    __HAL_TIM_SET_COMPARE(timer, hal_tim_channel, 0);
    LL_TIM_OC_EnablePreload(timer->Instance, ll_tim_channel);

    TIM_CCxChannelCmd(timer->Instance, hal_tim_channel, TIM_CCx_ENABLE);

    hal_dma_peripheral_increment_set_enabled(dma, false);
    hal_dma_memory_increment_set_enabled(dma, true);
    hal_dma_data_transfer_set_direction(dma, DMA_MEMORY_TO_PERIPH);
}

static void signal_line_set_to_rx(TIM_HandleTypeDef *timer,
                                  const struct io_bdshot_stm32_channel_config *channel_config)
{
    DMA_HandleTypeDef *dma = channel_config->dma;
    const uint32_t hal_tim_channel = channel_config->timer_channel;
    const uint32_t ll_tim_channel = hal_to_ll_tim_channel(hal_tim_channel);

    __HAL_DMA_DISABLE(dma);

    LL_TIM_EnableARRPreload(timer->Instance);
    LL_TIM_SetAutoReload(timer->Instance, UINT32_MAX);

    TIM_CCxChannelCmd(timer->Instance, hal_tim_channel, TIM_CCx_DISABLE);

    LL_TIM_IC_Config(timer->Instance, ll_tim_channel,
                     LL_TIM_ACTIVEINPUT_DIRECTTI | LL_TIM_ICPSC_DIV1 | LL_TIM_IC_FILTER_FDIV1_N2 |
                         LL_TIM_IC_POLARITY_BOTHEDGE);

    TIM_CCxChannelCmd(timer->Instance, hal_tim_channel, TIM_CCx_ENABLE);

    hal_dma_peripheral_increment_set_enabled(dma, false);
    hal_dma_memory_increment_set_enabled(dma, true);
    hal_dma_data_transfer_set_direction(dma, DMA_PERIPH_TO_MEMORY);
}

static void on_dma_transfer_complete_isr(void *user_data)
{
    struct io_bdshot_stm32_channel_data *channel_data = user_data;

    const struct io_bdshot_stm32_config *config = channel_data->bdshot->config;
    const uint8_t channel_index = channel_data->channel_index;
    const struct io_bdshot_stm32_channel_config *channel_config = &config->channels[channel_index];

    TIM_HandleTypeDef *timer = config->timer;
    DMA_HandleTypeDef *dma = channel_config->dma;

    const uint32_t timer_channel = channel_config->timer_channel;
    const uint32_t dma_src = hal_get_tim_channel_dma_src(timer_channel);

    // Disable DMA requests from the timer peripheral to prevent
    // accidental transfers while we program the DMA
    __HAL_TIM_DISABLE_DMA(timer, dma_src);

    if (channel_data->direction == IO_BDSHOT_STM32_CHANNEL_DIRECTION_OUTPUT) {
        signal_line_set_to_rx(timer, channel_config);

        volatile uint32_t *ccrx = hal_get_tim_channel_ccrx_reg(timer, timer_channel);

        if (ccrx != NULL) {
            (void)HAL_DMA_Start_IT(dma, (uint32_t)ccrx, (uint32_t)channel_config->rx_buffer,
                                   RX_FRAME_SIZE);

            __HAL_TIM_ENABLE_DMA(timer, dma_src);

            channel_data->direction = IO_BDSHOT_STM32_CHANNEL_DIRECTION_INPUT;
        }
    } else {
        signal_line_set_to_tx(timer, channel_config);

        channel_data->direction = IO_BDSHOT_STM32_CHANNEL_DIRECTION_OUTPUT;
    }
}

static void set_frame(const io_bdshot_t *bdshot, uint8_t channel, uint16_t frame)
{
    const struct io_bdshot_stm32_config *config = bdshot->config;
    struct io_bdshot_stm32_data *data = bdshot->data;

    ULYSSES_ASSERT(channel < config->channel_count, "BDShot channel index is out of range");

    data->channels[channel].frame = frame;
    data->channels[channel].is_frame_dirty = true;
}

static void set_response(const io_bdshot_t *bdshot, uint8_t channel,
                         struct io_bdshot_response *response)
{
    const struct io_bdshot_stm32_config *config = bdshot->config;
    struct io_bdshot_stm32_data *data = bdshot->data;

    ULYSSES_ASSERT(channel < config->channel_count, "BDShot channel index is out of range");

    data->channels[channel].response = response;
}

static enum io_status set_active(const io_bdshot_t *bdshot, uint8_t channel, bool is_active)
{
    const struct io_bdshot_stm32_config *config = bdshot->config;
    struct io_bdshot_stm32_data *data = bdshot->data;

    ULYSSES_ASSERT(channel < config->channel_count, "BDShot channel index is out of range");

    const struct io_bdshot_stm32_channel_config *channel_config = &config->channels[channel];
    struct io_bdshot_stm32_channel_data *channel_data = &data->channels[channel];
    GPIO_TypeDef *gpio = channel_config->gpio;
    const uint32_t ll_gpio_pin = hal_to_ll_gpio_pin(channel_config->gpio_pin);
    HAL_StatusTypeDef status;

    if (is_active) {
        ll_gpio_set_af(gpio, ll_gpio_pin, channel_data->original_gpio_af);
        LL_GPIO_SetPinMode(gpio, ll_gpio_pin, LL_GPIO_MODE_ALTERNATE);

        status = HAL_TIM_PWM_Start(config->timer, channel_config->timer_channel);
    } else {
        status = HAL_TIM_PWM_Stop(config->timer, channel_config->timer_channel);

        LL_GPIO_ResetOutputPin(gpio, ll_gpio_pin);
        LL_GPIO_SetPinMode(gpio, ll_gpio_pin, LL_GPIO_MODE_OUTPUT);
    }

    if (status == HAL_OK) {
        channel_data->is_active = is_active;
    }

    return status == HAL_OK ? IO_STATUS_OK : IO_STATUS_ERROR;
}

static enum io_status transceive_start(const io_bdshot_t *bdshot)
{
    enum io_status status = IO_STATUS_OK;

    const struct io_bdshot_stm32_config *config = bdshot->config;
    struct io_bdshot_stm32_data *data = bdshot->data;

    TIM_HandleTypeDef *timer = config->timer;

    // Prevent the timer from trigger DMA in case there were channels that had a failed
    // transaction and never disabled itself
    __HAL_TIM_DISABLE_DMA(timer, data->active_dma_mask);

    // Rebuild active DMA sources since the some channels may no longer be active
    data->active_dma_mask = 0;

    for (uint8_t i = 0; i < config->channel_count; i++) {
        struct io_bdshot_stm32_channel_data *channel_data = &data->channels[i];

        if (!channel_data->is_active) {
            continue;
        }

        if (channel_data->direction != IO_BDSHOT_STM32_CHANNEL_DIRECTION_OUTPUT) {
            continue;
        }

        const struct io_bdshot_stm32_channel_config *channel_config = &config->channels[i];

        if (channel_data->is_frame_dirty) {
            channel_data->is_frame_dirty = false;
            frame_build_dma_tx_buffer(channel_config->tx_buffer, channel_data->frame);
        }

        // Write to the compare register for this timer channel to generate DShot PWM signal
        volatile uint32_t *ccrx =
            hal_get_tim_channel_ccrx_reg(timer, channel_config->timer_channel);

        if (ccrx == NULL) {
            status = IO_STATUS_ERROR;
            break;
        }

        (void)HAL_DMA_Start_IT(channel_config->dma, (uint32_t)channel_config->tx_buffer,
                               (uint32_t)ccrx, TX_FRAME_SIZE);

        data->active_dma_mask |= hal_get_tim_channel_dma_src(channel_config->timer_channel);
    }

    // Enable DMA transfers from the timers.
    // Do this separately from the DMA controller configuration
    // to ensure that transmissions on the same TIM instance are synchronized.
    if (status == IO_STATUS_OK && data->active_dma_mask != 0) {
        // Restore the free running timer ARR used for telemetry RX to a value for TX.
        // Must disable ARR preload here since the free running timer used for RX will
        // never have a UEV.
        LL_TIM_DisableARRPreload(timer->Instance);
        LL_TIM_SetAutoReload(timer->Instance, BIT_TICKS - 1);
        LL_TIM_EnableARRPreload(timer->Instance);

        LL_TIM_SetCounter(timer->Instance, 0);

        // Enable DMA requests from the timer peripheral
        __HAL_TIM_ENABLE_DMA(timer, data->active_dma_mask);
    }

    return status;
}

static enum io_status transceive_stop(const io_bdshot_t *bdshot)
{
    enum io_status status = IO_STATUS_OK;

    const struct io_bdshot_stm32_config *config = bdshot->config;
    struct io_bdshot_stm32_data *data = bdshot->data;
    TIM_HandleTypeDef *timer = config->timer;

    for (uint8_t i = 0; i < config->channel_count; i++) {
        const struct io_bdshot_stm32_channel_config *channel_config = &config->channels[i];
        struct io_bdshot_stm32_channel_data *channel_data = &data->channels[i];

        if (!channel_data->is_active) {
            continue;
        }

        ULYSSES_ASSERT(channel_data->response != NULL, "No location to store response");

        DMA_HandleTypeDef *dma = channel_config->dma;
        uint32_t received_edges = RX_FRAME_SIZE;

        // Wire bits is designed to be half the edge transitions as the GCR encoded
        // value. So for a fixed sized DMA transfer, it is not always going to complete,
        // so we have to abort it, but the data should still be good.
        if (channel_data->direction == IO_BDSHOT_STM32_CHANNEL_DIRECTION_INPUT) {
            uint32_t dma_src = hal_get_tim_channel_dma_src(channel_config->timer_channel);

            __HAL_TIM_DISABLE_DMA(timer, dma_src);

            uint32_t unreceived_edges = __HAL_DMA_GET_COUNTER(dma);

            // DMA peripheral provides the number of unreceived data units, each
            // data unit is an edge time
            received_edges = RX_FRAME_SIZE - unreceived_edges;

            (void)HAL_DMA_Abort(dma);

            signal_line_set_to_tx(timer, channel_config);

            channel_data->direction = IO_BDSHOT_STM32_CHANNEL_DIRECTION_OUTPUT;
        }

        bool is_success = wire_frame_decode_from_signal(&channel_data->response->wire_frame,
                                                        channel_config->rx_buffer, received_edges);

        channel_data->response->status = is_success ? IO_STATUS_OK : IO_STATUS_ERROR;
    }

    return status;
}

static const struct io_bdshot_ops BDSHOT_STM32_OPS = {
    .set_frame = set_frame,
    .set_response = set_response,
    .set_active = set_active,
    .transceive_start = transceive_start,
    .transceive_stop = transceive_stop,
};

void io_bdshot_stm32_init(io_bdshot_t *bdshot, const struct io_bdshot_stm32_config *config,
                          struct io_bdshot_stm32_data *data)
{
    ULYSSES_ASSERT(bdshot != NULL, "Instance must be non NULL");
    ULYSSES_ASSERT(config != NULL, "Config must be non NULL");
    ULYSSES_ASSERT(data != NULL, "Data must be non NULL");
    ULYSSES_ASSERT(config->timer != NULL, "Must provide a timer instance");
    ULYSSES_ASSERT(config->channels != NULL, "Must provide channel configs");
    ULYSSES_ASSERT(data->channels != NULL, "Must provide channel data");
    ULYSSES_ASSERT(config->channel_count > 0, "Must provide at least 1 channel");
    ULYSSES_ASSERT(config->channel_count <= IO_BDSHOT_STM32_MAX_CHANNELS, "Too many channels");

    data->active_dma_mask = 0;

    for (uint8_t i = 0; i < config->channel_count; i++) {
        const struct io_bdshot_stm32_channel_config *channel_config = &config->channels[i];
        struct io_bdshot_stm32_channel_data *channel_data = &data->channels[i];

        const uint32_t ll_gpio_pin = hal_to_ll_gpio_pin(channel_config->gpio_pin);

        (void)memset(channel_config->tx_buffer, 0,
                     IO_BDSHOT_STM32_TX_BUFFER_SIZE * sizeof(uint32_t));
        (void)memset(channel_config->rx_buffer, 0,
                     IO_BDSHOT_STM32_RX_BUFFER_SIZE * sizeof(uint32_t));

        // Needed so ISR can identify the channel
        channel_data->bdshot = bdshot;
        channel_data->channel_index = i;

        channel_data->is_active = false;
        channel_data->is_frame_dirty = false;
        channel_data->direction = IO_BDSHOT_STM32_CHANNEL_DIRECTION_OUTPUT;
        channel_data->original_gpio_af = ll_gpio_get_af(channel_config->gpio, ll_gpio_pin);

        data->active_dma_mask |= hal_get_tim_channel_dma_src(channel_config->timer_channel);

        io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(channel_config->dma),
                        IO_IRQ_EVENT_TRANSFER_COMPLETE, on_dma_transfer_complete_isr,
                        (void *)channel_data);
    }

    bdshot->data = data;
    bdshot->config = config;
    bdshot->ops = &BDSHOT_STM32_OPS;

    // Default to inactive for all channels
    for (uint8_t i = 0; i < config->channel_count; i++) {
        (void)set_active(bdshot, i, false);
    }
}
