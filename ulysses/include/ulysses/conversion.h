#ifndef ULYSSES_CONVERSION
#define ULYSSES_CONVERSION

#include <stddef.h>

/**
 * @brief Converts minutes to microseconds
 * 
 * @param minutes Minutes to convert
 *
 * @return Time in microseconds
 */
static inline float minutes_to_us(float minutes)
{
    return minutes * 60 * 1000000;
}

#endif // ULYSSES_CONVERSION
