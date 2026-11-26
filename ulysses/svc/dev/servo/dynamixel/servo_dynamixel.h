#ifndef ULYSSES_SVC_DEV_SERVO_DYNAMIXEL
#define ULYSSES_SVC_DEV_SERVO_DYNAMIXEL

#include <stdbool.h>
#include <stdint.h>

#include "ulysses/io/gpio.h"
#include "ulysses/io/uart.h"
#include "ulysses/lib/dynamixel/protocol2/parameters.h"

#include "dev/servo/servo.h" // IWYU pragma: keep

#define DYNAMIXEL_TICKS_PER_ROTATION (0x1000)
#define DYNAMIXEL_UART_TIMEOUT_MS (20)

/** @brief Fixed configuration for a Dynamixel servo bus */
struct dev_servo_dynamixel_config {
    const io_uart_dev_t *uart;                 ///< Dedicated, initialized packet-mode UART
    const io_gpio_dev_t *transmit_enable_gpio; ///< High for TX, low for RX
    const uint8_t *ids; ///< Static array of unique servo IDs (0 through 252) in position order
    uint32_t count;     ///< Nonzero servo count that fits a position Sync Write
    uint32_t sync_write_max_length; ///< Maximum Sync Write data bytes per servo, at least 4
    uint32_t packet_capacity;       ///< Packet buffer capacity in bytes
    uint16_t parameters_capacity;   ///< Parameter buffer capacity in bytes
};

/** @brief Statically allocated state for a Dynamixel servo bus */
struct dev_servo_dynamixel_data {
    float *positions; ///< Static feedback storage with one slot per servo
    struct dynamixel_protocol2_sync_write_device *write_devices; ///< Static array of count entries
    volatile bool busy;
    volatile dev_servo_read_cb_t read_cb;
    volatile dev_servo_write_cb_t write_cb;
    volatile io_uart_receive_cb_t receive_status_cb;
    void *volatile user_data;
    uint8_t *packet;     ///< Static UART packet buffer
    uint8_t *parameters; ///< Static buffer for request and reply parameters
};

/**
 * @brief Initializes a Dynamixel servo bus
 *
 * The UART and transmit-enable GPIO must already be initialized.
 * The servos must support Fast Sync Read and be configured for position control. The full
 * driver goal range requires Extended Position Control or Current-based Position Control.
 * The calling task and UART interrupts must run on the same core.
 *
 * The ids, positions, and write_devices arrays must have count entries. Each write device must
 * point to its own writable static buffer of sync_write_max_length bytes. The separate packet and
 * parameters buffers must fit the bus requests and replies. Init checks the capacities in config.
 *
 * @param[out] servo Static servo bus object to initialize
 * @param config Static UART, transmit-enable GPIO, servo IDs, and buffer capacities
 * @param[in,out] data Static bus storage with the buffer pointers set
 */
void dev_servo_dynamixel_init(dev_servo_t *servo, const struct dev_servo_dynamixel_config *config,
                              struct dev_servo_dynamixel_data *data);

enum uls_status dev_servo_dynamixel_ping_all(dev_servo_t *servo);

enum uls_status dev_servo_dynamixel_set_torque(dev_servo_t *servo, enum dev_servo_torque_mode mode);

enum uls_status dev_servo_dynamixel_read_positions(dev_servo_t *servo, float *positions);

enum uls_status dev_servo_dynamixel_read_positions_async(dev_servo_t *servo, dev_servo_read_cb_t cb,
                                                         void *user_data);

enum uls_status dev_servo_dynamixel_write_positions(dev_servo_t *servo, const float *positions);

enum uls_status dev_servo_dynamixel_write_positions_async(dev_servo_t *servo,
                                                          const float *positions,
                                                          dev_servo_write_cb_t cb, void *user_data);

#endif // ULYSSES_SVC_DEV_SERVO_DYNAMIXEL
