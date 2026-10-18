// SPI driver public interface.
//
// The driver knows nothing about which devices exist on the board. Callers
// describe each device (SPI peripheral, chip-select pin) at init time and then
// pass the device handle to every transfer.

#ifndef ULYSSES_IO_SPI
#define ULYSSES_IO_SPI

#include <stddef.h>
#include <stdint.h>

// Result of io_spi_transfer().
//
// Only failures that can happen at run time are reported here. Programming
// errors (NULL pointers, zero length, ...) trigger ULYSSES_ASSERT instead.
enum io_spi_status {
    IO_SPI_OK = 0,      // success
    IO_SPI_ERR_TIMEOUT, // transfer did not finish within the timeout
    IO_SPI_ERR_BUSY,    // peripheral is already in use
    IO_SPI_ERR_BUS,     // hardware error reported by the peripheral
};

// One SPI device: an SPI peripheral plus its own chip-select pin.
//
// Opaque: the contents are private to spi.c. Get one from io_spi_device_init().
typedef struct io_spi_device io_spi_device_t;

// Register one SPI device and get a handle for it.
//
// Call once per device at startup, before any io_spi_transfer() on it. Not
// thread-safe: do all device inits from a single context.
//
// hw_handle: platform SPI handle the device is wired to (e.g. a HAL SPI
//            handle). Must not be NULL.
// cs_port:   platform GPIO port of the chip-select pin. Must not be NULL.
// cs_pin:    chip-select pin within cs_port.
//
// The chip-select pin is driven high (device not selected) before returning.
// Asserts if a pointer is NULL or no free device slot is left.
io_spi_device_t *io_spi_device_init(void *hw_handle, void *cs_port, uint16_t cs_pin);

// Exchange len bytes with a device, waiting until the transfer is done.
//
// Chip select is pulled low for the whole transfer and released afterwards,
// also when the transfer fails. SPI sends and receives at the same time, so
// rx[i] is the byte that arrived while tx[i] was being sent.
//
// device:     handle from io_spi_device_init(). Must not be NULL.
// tx:         len bytes to send. Must not be NULL.
// rx:         buffer for len received bytes, or NULL to discard them.
// len:        number of bytes to exchange, from 1 to 65535.
// timeout_ms: how long to wait before giving up, in milliseconds.
//
// Do not call from an interrupt handler. Not thread-safe: do not use devices
// on the same SPI peripheral from two contexts at once.
//
// Asserts if device or tx is NULL or len is out of range. Returns IO_SPI_OK,
// IO_SPI_ERR_TIMEOUT, IO_SPI_ERR_BUSY or IO_SPI_ERR_BUS.
enum io_spi_status io_spi_transfer(io_spi_device_t *device, const uint8_t *tx, uint8_t *rx,
                                   size_t len, uint32_t timeout_ms);

#endif // ULYSSES_IO_SPI
