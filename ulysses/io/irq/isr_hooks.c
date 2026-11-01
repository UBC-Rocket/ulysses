#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "ulysses/io/irq.h"

// TODO: once the IOC is finalized, define the remaining HAL callbacks here and
// TODO: signal the appropriate interrupt

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    io_irq_signal(IO_IRQ_HAL_HANDLE_TO_ID(hspi), IO_IRQ_EVENT_TX_RX_COMPLETE);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    io_irq_signal(IO_IRQ_HAL_HANDLE_TO_ID(hspi), IO_IRQ_EVENT_TX_COMPLETE);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    io_irq_signal(IO_IRQ_HAL_HANDLE_TO_ID(hspi), IO_IRQ_EVENT_ERROR);
}
