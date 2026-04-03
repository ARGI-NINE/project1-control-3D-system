#include "axis_config.h"

const slave_axis_config_t *slave_axis_config_get(void) {
  static slave_axis_config_t cfg;
  static bool inited = false;
  if (!inited) {
    axis_t axis = (axis_t)SLAVE_AXIS_INDEX;
    if ((int)axis < 0 || axis > AXIS_ROLL) {
      axis = AXIS_PITCH;
    }
    const axis_params_t *p = axis_params_get(axis);
    cfg.axis = axis;
    cfg.axis_id = p->axis_id;
    cfg.axis_sign = p->axis_sign;
    cfg.servo_zero_deg = p->servo_zero_deg;
    cfg.pos_min_deg = p->pos_min_deg;
    cfg.pos_max_deg = p->pos_max_deg;
    cfg.pulse_min_us = p->pulse_min_us;
    cfg.pulse_max_us = p->pulse_max_us;
    cfg.slew_rate_deg_s = p->slew_rate_deg_s;
    cfg.acc_limit_deg_s2 = p->acc_limit_deg_s2;
    cfg.pwm_hz = SLAVE_SERVO_PWM_HZ;
    cfg.timeout_ms = SLAVE_TIMEOUT_MS;
    cfg.return_to_center_on_timeout = (SLAVE_RETURN_CENTER_ON_TIMEOUT != 0u);
    cfg.timeout_return_time_s = SLAVE_TIMEOUT_RETURN_S;
    cfg.vbat_low_v = SLAVE_VBAT_LOW_V;
    inited = true;
  }
  return &cfg;
}
