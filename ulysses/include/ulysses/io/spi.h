/*
 * SPI driver public interface.
 *
 * The driver knows nothing about which devices exist on the board. Callers
 * describe each device (SPI peripheral, chip-select pin) at init time and then
 * pass the device handle to every transfer.
 */

#ifndef ULYSSES_IO_SPI_H
#define ULYSSES_IO_SPI_H

#include <stdint.h>

/*
 * Result of an SPI driver call.
 *
 * These are SPI-specific for now.
 */
typedef enum {
    SPI_OK = 0,       /* success */
    SPI_ERR_PARAM,    /* invalid argument (NULL pointer, zero length, ...) */
    SPI_ERR_TIMEOUT,  /* transfer did not finish within the timeout */
    SPI_ERR_BUSY,     /* peripheral is already in use */
    SPI_ERR_BUS,      /* hardware error reported by the peripheral */
    SPI_ERR_NO_SPACE, /* no free slot left for another device */
} spi_status_t;

/*
 * One SPI device: an SPI peripheral plus its own chip-select pin.
 *
 * Opaque: the contents are private to spi.c. Get one from spi_device_init().
 */
typedef struct spi_device spi_device_t;

#endif /* ULYSSES_IO_SPI_H */
