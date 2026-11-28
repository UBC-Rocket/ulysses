#ifndef ULYSSES_LIB_TIMESTAMP
#define ULYSSES_LIB_TIMESTAMP

#include <stdint.h>

/** @brief Callback that returns a nondecreasing 64-bit timestamp in microseconds */
typedef uint64_t (*source_cb_t)(void);

/**
 * @brief Reads the current timestamp in microseconds
 *
 * @return The current timestamp in microseconds.
 */
uint64_t timestamp_now_us(void);

/**
 * @brief Reads the current timestamp in milliseconds
 *
 * @return The current timestamp in milliseconds.
 */
uint64_t timestamp_now_ms(void);

/**
 * @brief Sets the microsecond precision source for subsequent timestamp reads
 *
 * @param source_us Timestamp source callback
 */
void timestamp_set_source(source_cb_t source_us);

#endif // ULYSSES_LIB_TIMESTAMP
