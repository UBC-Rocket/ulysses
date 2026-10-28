#include "stm32h7xx_hal.h"

#include "ulysses/io/irq.h"

// TODO: once the IOC is finalized, define the remaining HAL callbacks here and
// TODO: signal the appropriate interrupt

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    io_irq_signal(IO_IRQ_HAL_HANDLE_TO_ID(hi2c), IO_IRQ_EVENT_TX_COMPLETE);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    io_irq_signal(IO_IRQ_HAL_HANDLE_TO_ID(hi2c), IO_IRQ_EVENT_RX_COMPLETE);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    io_irq_signal(IO_IRQ_HAL_HANDLE_TO_ID(hi2c), IO_IRQ_EVENT_ERROR);
}
