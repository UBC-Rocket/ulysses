#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stddef.h>
#include <stdint.h>

#ifdef DEBUG
void debug_uart_init(void);
void debug_uart_write(const uint8_t *data, size_t length);
void debug_uart_write_string(const char *str);
#else
static inline void debug_uart_init(void) {}
static inline void debug_uart_write(const uint8_t *data, size_t length)
{
    (void)data;
    (void)length;
}
static inline void debug_uart_write_string(const char *str)
{
    (void)str;
}
#endif

#endif /* DEBUG_UART_H */
