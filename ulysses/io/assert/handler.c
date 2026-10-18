#include <stdbool.h>

#include "cmsis_compiler.h" // IWYU pragma: keep

#include "ulysses/compiler.h"
#include "ulysses/lib/assert.h"

ULYSSES_NORETURN void ulysses_assert_handler(ULYSSES_UNUSED const char *file,
                                             ULYSSES_UNUSED uint64_t line,
                                             ULYSSES_UNUSED const char *message)
{
    __disable_irq();

    __BKPT(0);

    while (true) {
        // Spin forever
    }
}
