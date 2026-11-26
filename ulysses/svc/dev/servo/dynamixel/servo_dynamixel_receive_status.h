#ifndef ULYSSES_SVC_DEV_SERVO_DYNAMIXEL_RECEIVE_STATUS
#define ULYSSES_SVC_DEV_SERVO_DYNAMIXEL_RECEIVE_STATUS

#include "ulysses/io/uart.h"
#include "ulysses/status.h"
#include "ulysses/svc/dev/servo.h"

// Receive the raw status packet into the bus buffer.
enum uls_status dev_servo_dynamixel_receive_status(const dev_servo_t *servo);

// Callback context is servo. A prefix start failure is returned. A body start failure calls cb.
enum uls_status dev_servo_dynamixel_receive_status_async(dev_servo_t *servo,
                                                         io_uart_receive_cb_t cb);

#endif // ULYSSES_SVC_DEV_SERVO_DYNAMIXEL_RECEIVE_STATUS
