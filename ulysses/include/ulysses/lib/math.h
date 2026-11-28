#ifndef ULYSSES_LIB_MATH
#define ULYSSES_LIB_MATH

#include <stddef.h>

static inline size_t min(size_t a, size_t b)
{
    return a < b ? a : b;
}

static inline size_t max(size_t a, size_t b)
{
    return a > b ? a : b;
}

static inline size_t clamp(size_t value, size_t lower, size_t upper)
{
    return max(lower, min(value, upper));
}

#endif // ULYSSES_LIB_MATH
