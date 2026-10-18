#ifndef ULYSSES_IO_UART
#define ULYSSES_IO_UART

#include <stddef.h>
#include <stdint.h>

typedef enum { // FIXME: this is copied from old code, maybe trim down possible values? or have separate status per module
    IO_OK = 0,
    IO_ERR_BUSY,      ///< queue full / channel still transmitting (try again)
    IO_ERR_TIMEOUT,   ///< blocking call did not complete in time
    IO_ERR_NACK,      ///< target did not acknowledge
    IO_ERR_BUS,       ///< bus-level fault (DMA error, framing, etc.)
    IO_ERR_NO_DEVICE, ///< @p dev not present / @p id out of range
    IO_ERR_PARAM,     ///< bad argument
    IO_ERR_NOT_READY, ///< called before init or in the wrong mode
} io_status_t;

typedef struct io_uart io_uart_t;

typedef void (*io_uart_recv_cb)(const uint8_t *buf, uint16_t len);
typedef void (*io_uart_send_cb)(uint16_t xferd);

io_uart_t *io_uart_init(void);

#endif // ULYSSES_IO_UART
