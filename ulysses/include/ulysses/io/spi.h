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

/*
 * Register one SPI device and get a handle for it.
 *
 * Call once per device at startup, before any spi_transfer() on it. Not
 * thread-safe: do all device inits from a single context.
 *
 * out_device: receives the device handle on success. Must not be NULL.
 * hw_handle:  platform SPI handle the device is wired to (e.g. a HAL SPI
 *             handle). Must not be NULL.
 * cs_port:    platform GPIO port of the chip-select pin. Must not be NULL.
 * cs_pin:     chip-select pin within cs_port.
 *
 * The chip-select pin is driven high (device not selected) before returning.
 *
 * Returns SPI_OK, SPI_ERR_PARAM if a pointer is NULL, or SPI_ERR_NO_SPACE if
 * no free device slot is left.
 */
spi_status_t spi_device_init(spi_device_t **out_device, void *hw_handle, void *cs_port,
                             uint16_t cs_pin);

#endif /* ULYSSES_IO_SPI_H */
