#ifndef ULYSSES_LIB_TIMESTAMP
#define ULYSSES_LIB_TIMESTAMP

#include <stdint.h>

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
 * @param source Function that returns a nondecreasing 64-bit microsecond timestamp
 */
void timestamp_set_source(uint64_t (*source_us)(void));

#endif // ULYSSES_LIB_TIMESTAMP
