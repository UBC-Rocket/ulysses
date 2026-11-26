#ifndef ULYSSES_SVC_DEV_SERVO_DYNAMIXEL_RECEIVE_STATUS
#define ULYSSES_SVC_DEV_SERVO_DYNAMIXEL_RECEIVE_STATUS

#include "ulysses/io/uart.h"
#include "ulysses/status.h"
#include "ulysses/svc/dev/servo.h"

// Switch to RX and receive a raw reply into the bus packet buffer for the caller to validate.
enum uls_status dev_servo_dynamixel_receive_status(const dev_servo_t *servo);

// Like the blocking receiver, with servo as the callback context.
// A prefix start failure is returned. A body start failure calls cb.
enum uls_status dev_servo_dynamixel_receive_status_async(dev_servo_t *servo,
                                                         io_uart_receive_cb_t cb);

#endif // ULYSSES_SVC_DEV_SERVO_DYNAMIXEL_RECEIVE_STATUS
