#ifndef ULYSSES_IO_DELAY
#define ULYSSES_IO_DELAY

#include <stdint.h>

/**
 * @brief Busy-waits for at least the given time
 *
 * For code that runs before the scheduler starts, tasks should use osDelay() instead. Must not be
 * called from an interrupt.
 *
 * @param ms Time to wait in milliseconds
 */
void io_spin_delay_ms(uint32_t ms);

#endif // ULYSSES_IO_DELAY
