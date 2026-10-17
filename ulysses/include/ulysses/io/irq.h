#ifndef ULYSSES_IO_IRQ
#define ULYSSES_IO_IRQ

#include <stdint.h>

#define IO_IRQ_HAL_HANDLE_TO_ID(handle_ptr) ((uint64_t)(handle_ptr))

enum io_irq_event {
    IO_IRQ_EVENT_TRIGGER,
    IO_IRQ_EVENT_TX_COMPLETE,
    IO_IRQ_EVENT_RX_COMPLETE,
    IO_IRQ_EVENT_TX_RX_COMPLETE,
    IO_IRQ_EVENT_ERROR,
    IO_IRQ_EVENT_COUNT,
};

typedef void (*io_irq_handler_t)(void *user_data);

void io_irq_register(uint64_t id, enum io_irq_event event, io_irq_handler_t handler,
                     void *user_data);

void io_irq_signal(uint64_t id, enum io_irq_event event);

#endif // ULYSSES_IO_IRQ
