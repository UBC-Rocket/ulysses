#include <stdbool.h>
#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/irq.h"

#define MAX_EVENT_HANDLER_ENTRIES (32)

struct irq_handler_entry {
    uint64_t id;
    void *user_data;
    io_irq_handler_t handler;
};

static struct irq_handler_entry event_handlers[IO_IRQ_EVENT_COUNT][MAX_EVENT_HANDLER_ENTRIES];

void io_irq_register(uint64_t id, enum io_irq_event event, io_irq_handler_t handler,
                     void *user_data)
{
    ULYSSES_ASSERT(event < IO_IRQ_EVENT_COUNT, "Invalid IRQ event");
    ULYSSES_ASSERT(handler != NULL, "IRQ handler must be non NULL");

    bool is_success = false;

    struct irq_handler_entry *handlers = event_handlers[event];

    for (uint32_t i = 0; i < MAX_EVENT_HANDLER_ENTRIES; i++) {
        struct irq_handler_entry *handler_slot = &handlers[i];

        if (handler_slot->handler == NULL) {
            handler_slot->id = id;
            handler_slot->user_data = user_data;
            handler_slot->handler = handler;

            is_success = true;
            break;
        }
    }

    ULYSSES_ASSERT(is_success, "Exceeded maximum number of IRQ handlers");
}

void io_irq_signal(uint64_t id, enum io_irq_event event)
{
    ULYSSES_ASSERT(event < IO_IRQ_EVENT_COUNT, "Invalid IRQ event");

    struct irq_handler_entry *handlers = event_handlers[event];

    for (uint32_t i = 0; i < MAX_EVENT_HANDLER_ENTRIES; i++) {
        struct irq_handler_entry *entry = &handlers[i];

        if (entry->handler != NULL && entry->id == id) {
            entry->handler(entry->user_data);
        }
    }
}
