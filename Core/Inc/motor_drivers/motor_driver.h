#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float servo_yl;     /* servo 1 — yaw left  [0..100] % */
    float servo_yr;     /* servo 2 — yaw right [0..100] % */
    float motor_top;    /* motor 1 — top       [0..100] % */
    float motor_bot;    /* motor 2 — bottom    [0..100] % */
} control_out_t;

void motor_driver_init(void);
void motor_driver_arm(uint32_t now_ms);
void motor_driver_disarm(void);
void motor_driver_set(const control_out_t *cmd, uint32_t now_ms);
void motor_driver_update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
