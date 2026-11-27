#ifndef ULYSSES_SVC_DEV_SERVO_DYNAMIXEL_PREPARE_INSTRUCTION
#define ULYSSES_SVC_DEV_SERVO_DYNAMIXEL_PREPARE_INSTRUCTION

#include <stdint.h>

#include "ulysses/lib/dynamixel/protocol2/parameters.h"
#include "ulysses/svc/dev/servo.h"

uint16_t dev_servo_dynamixel_prepare_fast_sync_read(
    const dev_servo_t *servo, const struct dynamixel_protocol2_fast_sync_read_request *request);

uint16_t dev_servo_dynamixel_prepare_sync_write(
    const dev_servo_t *servo, const struct dynamixel_protocol2_sync_write_request *request);

#endif // ULYSSES_SVC_DEV_SERVO_DYNAMIXEL_PREPARE_INSTRUCTION
