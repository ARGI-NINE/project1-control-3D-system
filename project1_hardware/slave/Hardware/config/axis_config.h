#ifndef SLAVE_AXIS_CONFIG_H
#define SLAVE_AXIS_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/include/axis_params.h"
#include "../common/include/project_config.h"

typedef struct {
  axis_t axis;
  uint8_t axis_id;
  float axis_sign;
  float servo_zero_deg;
  float pos_min_deg;
  float pos_max_deg;
  uint16_t pulse_min_us;
  uint16_t pulse_max_us;
  float slew_rate_deg_s;
  float acc_limit_deg_s2;
  uint16_t pwm_hz;
  uint32_t timeout_ms;
  bool return_to_center_on_timeout;
  float timeout_return_time_s;
  float vbat_low_v;
} slave_axis_config_t;

const slave_axis_config_t *slave_axis_config_get(void);

#endif /* SLAVE_AXIS_CONFIG_H */
