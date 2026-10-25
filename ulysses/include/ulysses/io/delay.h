#ifndef ULYSSES_IO_DELAY
#define ULYSSES_IO_DELAY

#include <stdint.h>

/**
 * @brief Blocks the caller for at least the given time
 *
 * Inside a task, other tasks run while waiting. Must not be called from an interrupt.
 *
 * @param ms Time to wait in milliseconds
 */
void io_delay_ms(uint32_t ms);

#endif // ULYSSES_IO_DELAY
