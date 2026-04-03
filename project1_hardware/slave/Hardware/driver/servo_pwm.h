#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <stdint.h>

void servo_pwm_init(uint16_t pwm_hz);
void servo_pwm_set_pulse_us(uint16_t us);

/* Runtime state query */
uint16_t servo_pwm_get_pulse_us(void);
uint16_t servo_pwm_get_pwm_hz(void);

#endif /* SERVO_PWM_H */

