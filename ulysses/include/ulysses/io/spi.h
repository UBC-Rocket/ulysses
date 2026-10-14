/*
 * SPI driver public interface.
 *
 * The driver knows nothing about which devices exist on the board. Callers
 * describe each bus and device (SPI peripheral, chip-select pin) at init time
 * and then pass the device handle to every transfer.
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
    SPI_OK = 0,      /* success */
    SPI_ERR_PARAM,   /* invalid argument (NULL pointer, zero length, ...) */
    SPI_ERR_TIMEOUT, /* transfer did not finish within the timeout */
    SPI_ERR_BUSY,    /* peripheral is already in use */
    SPI_ERR_BUS,     /* hardware error reported by the peripheral */
} spi_status_t;

/*
 * One SPI peripheral (for example SPI1).
 *
 * The caller owns the memory and fills it with spi_bus_init(). Do not access
 * the fields directly.
 */
typedef struct {
    void *handle; /* platform SPI handle (e.g. a HAL SPI handle), only used by spi.c */
} spi_bus_t;

/*
 * One device on an SPI bus, selected by its own chip-select pin.
 *
 * Several devices can share one bus. The caller owns the memory and fills it
 * with spi_device_init(). Do not access the fields directly.
 */
typedef struct {
    spi_bus_t *bus;  /* bus the device is wired to */
    void *cs_port;   /* platform GPIO port of the chip-select pin, only used by spi.c */
    uint16_t cs_pin; /* chip-select pin within cs_port */
} spi_device_t;

#endif /* ULYSSES_IO_SPI_H */
