#ifndef ULYSSES_SVC_DEV_SERVO_DYNAMIXEL
#define ULYSSES_SVC_DEV_SERVO_DYNAMIXEL

#include <stdbool.h>
#include <stdint.h>

#include "ulysses/io/uart.h"
#include "ulysses/lib/dynamixel/protocol2_instructions.h"
#include "ulysses/svc/dev/servo.h"

// Largest XM430 position group supported by the Protocol 2.0 request encoder.
#define DEV_SERVO_DYNAMIXEL_PACKET_CAPACITY (160)

/** @brief Fixed configuration for a Dynamixel servo bus */
struct dev_servo_dynamixel_config {
    const io_uart_dev_t *uart; ///< Dedicated, initialized packet-mode UART
    const uint8_t *ids; ///< Static array of unique servo IDs (0 through 252) in position order
    uint32_t count;     ///< Number of configured servo IDs
};

/** @brief Statically allocated state for a Dynamixel servo bus */
struct dev_servo_dynamixel_data {
    float *positions; ///< Static feedback storage with one slot per servo
    struct dynamixel_protocol2_sync_write_device *write_devices; ///< Static array of count entries
    volatile bool busy;
    volatile dev_servo_read_cb_t read_cb;
    volatile dev_servo_write_cb_t write_cb;
    volatile io_uart_receive_cb_t reply_cb; ///< Completes a packet read or reports a transfer error
    void *volatile user_data;
    uint16_t reply_length;
    uint8_t packet[DEV_SERVO_DYNAMIXEL_PACKET_CAPACITY];
    uint8_t parameters[DEV_SERVO_DYNAMIXEL_PACKET_CAPACITY];
};

/**
 * @brief Binds a servo bus to a Dynamixel Protocol 2.0 UART transport
 *
 * Call once per bus during initialization, before starting servo transactions. This function
 * takes ownership of the UART callbacks. The UART must already handle half-duplex bus turnaround
 * and the XM430s must be configured for the intended operating mode and support Fast Sync Read.
 * The owning task and UART interrupts must run on the same core.
 *
 * @param[out] servo Static servo bus object to initialize
 * @param config Static UART and servo ID configuration
 * @param[in,out] data Static bus storage with positions and write_devices set
 */
void dev_servo_dynamixel_init(dev_servo_t *servo, const struct dev_servo_dynamixel_config *config,
                              struct dev_servo_dynamixel_data *data);

#endif // ULYSSES_SVC_DEV_SERVO_DYNAMIXEL
