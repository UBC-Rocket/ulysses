#ifndef ULYSSES_LIB_ASSERT
#define ULYSSES_LIB_ASSERT

#include <stdint.h>

#include "ulysses/compiler.h"

#define ULYSSES_ASSERT(cond, message)                            \
    do {                                                         \
        if (!(cond)) {                                           \
            ulysses_assert_handler(__FILE__, __LINE__, message); \
        }                                                        \
    } while (0)

void ulysses_assert_handler(const char *file, uint64_t line, const char *message) ULYSSES_NORETURN;

#endif // ULYSSES_LIB_ASSERT
