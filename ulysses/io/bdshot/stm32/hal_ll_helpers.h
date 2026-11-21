#ifndef ULYSSES_IO_BDSHOT_STM32_HAL_LL_HELPERS_H
#define ULYSSES_IO_BDSHOT_STM32_HAL_LL_HELPERS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h" // IWYU pragma: keep
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_tim.h"

#include "ulysses/lib/assert.h"

static inline uint32_t hal_to_ll_gpio_pin(uint32_t pin)
{
    uint32_t ll_pin = 0;

    switch (pin) {
    case GPIO_PIN_0:
        ll_pin = LL_GPIO_PIN_0;
        break;
    case GPIO_PIN_1:
        ll_pin = LL_GPIO_PIN_1;
        break;
    case GPIO_PIN_2:
        ll_pin = LL_GPIO_PIN_2;
        break;
    case GPIO_PIN_3:
        ll_pin = LL_GPIO_PIN_3;
        break;
    case GPIO_PIN_4:
        ll_pin = LL_GPIO_PIN_4;
        break;
    case GPIO_PIN_5:
        ll_pin = LL_GPIO_PIN_5;
        break;
    case GPIO_PIN_6:
        ll_pin = LL_GPIO_PIN_6;
        break;
    case GPIO_PIN_7:
        ll_pin = LL_GPIO_PIN_7;
        break;
    case GPIO_PIN_8:
        ll_pin = LL_GPIO_PIN_8;
        break;
    case GPIO_PIN_9:
        ll_pin = LL_GPIO_PIN_9;
        break;
    case GPIO_PIN_10:
        ll_pin = LL_GPIO_PIN_10;
        break;
    case GPIO_PIN_11:
        ll_pin = LL_GPIO_PIN_11;
        break;
    case GPIO_PIN_12:
        ll_pin = LL_GPIO_PIN_12;
        break;
    case GPIO_PIN_13:
        ll_pin = LL_GPIO_PIN_13;
        break;
    case GPIO_PIN_14:
        ll_pin = LL_GPIO_PIN_14;
        break;
    case GPIO_PIN_15:
        ll_pin = LL_GPIO_PIN_15;
        break;
    default:
        ULYSSES_ASSERT(false, "Unknown HAL GPIO pin");
        break;
    }

    return ll_pin;
}

static inline uint32_t hal_to_ll_tim_channel(uint32_t channel)
{
    uint32_t ll_channel = 0;

    switch (channel) {
    case TIM_CHANNEL_1:
        ll_channel = LL_TIM_CHANNEL_CH1;
        break;
    case TIM_CHANNEL_2:
        ll_channel = LL_TIM_CHANNEL_CH2;
        break;
    case TIM_CHANNEL_3:
        ll_channel = LL_TIM_CHANNEL_CH3;
        break;
    case TIM_CHANNEL_4:
        ll_channel = LL_TIM_CHANNEL_CH4;
        break;
    default:
        ULYSSES_ASSERT(false, "Unknown HAL TIM channel");
        break;
    }

    return ll_channel;
}

static inline uint32_t ll_gpio_get_af(GPIO_TypeDef *gpio, uint32_t pin)
{
    uint32_t af = 0;

    if (pin <= LL_GPIO_PIN_7) {
        af = LL_GPIO_GetAFPin_0_7(gpio, pin);
    } else {
        af = LL_GPIO_GetAFPin_8_15(gpio, pin);
    }

    return af;
}

static inline void ll_gpio_set_af(GPIO_TypeDef *gpio, uint32_t pin, uint32_t af)
{
    if (pin <= LL_GPIO_PIN_7) {
        LL_GPIO_SetAFPin_0_7(gpio, pin, af);
    } else {
        LL_GPIO_SetAFPin_8_15(gpio, pin, af);
    }
}

static inline volatile uint32_t *hal_get_tim_channel_ccrx_reg(TIM_HandleTypeDef *tim,
                                                              uint32_t hal_tim_channel)
{
    volatile uint32_t *ccrx = NULL;

    switch (hal_tim_channel) {
    case TIM_CHANNEL_1:
        ccrx = &tim->Instance->CCR1;
        break;
    case TIM_CHANNEL_2:
        ccrx = &tim->Instance->CCR2;
        break;
    case TIM_CHANNEL_3:
        ccrx = &tim->Instance->CCR3;
        break;
    case TIM_CHANNEL_4:
        ccrx = &tim->Instance->CCR4;
        break;
    default:
        break;
    }

    return ccrx;
}

static inline uint32_t hal_get_tim_channel_dma_src(uint32_t hal_tim_channel)
{
    uint32_t dma_source = 0;

    switch (hal_tim_channel) {
    case TIM_CHANNEL_1:
        dma_source = TIM_DMA_CC1;
        break;
    case TIM_CHANNEL_2:
        dma_source = TIM_DMA_CC2;
        break;
    case TIM_CHANNEL_3:
        dma_source = TIM_DMA_CC3;
        break;
    case TIM_CHANNEL_4:
        dma_source = TIM_DMA_CC4;
        break;
    default:
        ULYSSES_ASSERT(false, "Unknown HAL TIM channel");
        break;
    }

    return dma_source;
}

static inline void hal_dma_peripheral_increment_set_enabled(DMA_HandleTypeDef *dma, bool is_enabled)
{
    MODIFY_REG(((DMA_Stream_TypeDef *)dma->Instance)->CR, DMA_SxCR_PINC,
               is_enabled ? LL_DMA_PERIPH_INCREMENT : LL_DMA_PERIPH_NOINCREMENT);
}

static inline void hal_dma_memory_increment_set_enabled(DMA_HandleTypeDef *dma, bool is_enabled)
{
    MODIFY_REG(((DMA_Stream_TypeDef *)dma->Instance)->CR, DMA_SxCR_MINC,
               is_enabled ? LL_DMA_MEMORY_INCREMENT : LL_DMA_MEMORY_NOINCREMENT);
}

static inline void hal_dma_data_transfer_set_direction(DMA_HandleTypeDef *dma, uint32_t direction)
{
    uint32_t ll_direction = 0;

    switch (direction) {
    case DMA_PERIPH_TO_MEMORY:
        ll_direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
        break;
    case DMA_MEMORY_TO_PERIPH:
        ll_direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        break;
    case DMA_MEMORY_TO_MEMORY:
        ll_direction = LL_DMA_DIRECTION_MEMORY_TO_MEMORY;
        break;
    default:
        ULYSSES_ASSERT(false, "Unknown HAL DMA direction");
        break;
    }

    // Hack to get HAL to work with what we are doing, without
    // fully using LL
    dma->Init.Direction = direction;
    MODIFY_REG(((DMA_Stream_TypeDef *)dma->Instance)->CR, DMA_SxCR_DIR, ll_direction);
}

#endif // ULYSSES_IO_BDSHOT_STM32_HAL_LL_HELPERS_H
