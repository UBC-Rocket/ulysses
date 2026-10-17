#include <stdbool.h>

#include "ulysses/lib/assert.h"
#include "ulysses/compiler.h"

ULYSSES_WEAK ULYSSES_NORETURN void ulysses_assert_handler(ULYSSES_UNUSED const char *file,
                                                          ULYSSES_UNUSED uint64_t line,
                                                          ULYSSES_UNUSED const char *message)
{
    // This function should not be modified.

    // Instead, users should redefine this function without the weak attribute
    // and put architecture specific handling in that function.

    while (true) {
        // Spin forever
    }
}
