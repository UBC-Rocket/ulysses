#ifndef ULYSSES_IO_IRQ
#define ULYSSES_IO_IRQ

#include <stdint.h>

#define IO_IRQ_HAL_HANDLE_TO_ID(handle_ptr) ((uint64_t)(uintptr_t)(handle_ptr))

enum io_irq_event {
    IO_IRQ_EVENT_TRIGGER,
    IO_IRQ_EVENT_TX_COMPLETE,
    IO_IRQ_EVENT_RX_COMPLETE,
    IO_IRQ_EVENT_TX_RX_COMPLETE,
    IO_IRQ_EVENT_ERROR,
    IO_IRQ_EVENT_COUNT,
};

typedef void (*io_irq_handler_t)(void *user_data);

/**
 * @brief Registers a handler for an interrupt request
 * 
 * @param id Identifier of the interrupt request source
 * @param event Event associated with the interrupt request
 * @param handler Function to be called when receiving an interrupt request
 * @param user_data Data to be passed to the handler
 */
void io_irq_register(uint64_t id, enum io_irq_event event, io_irq_handler_t handler,
                     void *user_data);

/**
 * @brief Signals that an interrupt has occured
 * 
 * @param id Identifier of the interrupt request source
 * @param event Event associated with the interrupt request
 */
void io_irq_signal(uint64_t id, enum io_irq_event event);

#endif // ULYSSES_IO_IRQ
