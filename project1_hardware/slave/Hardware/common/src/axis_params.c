#include "../include/axis_params.h"

static const axis_params_t k_axis_table[3] = {
    /* Yaw */
    {.axis_id = 1,
     .axis_sign = 1.0f,
     .servo_zero_deg = 0.0f,
     .pos_min_deg = -90.0f,
     .pos_max_deg = 90.0f,
     .pulse_min_us = 1000,
     .pulse_max_us = 2000,
     .slew_rate_deg_s = 400.0f,
     .acc_limit_deg_s2 = 1800.0f},
    /* Pitch */
    {.axis_id = 2,
     .axis_sign = 1.0f,
     .servo_zero_deg = 0.0f,
     .pos_min_deg = -60.0f,
     .pos_max_deg = 60.0f,
     .pulse_min_us = 1000,
     .pulse_max_us = 2000,
     .slew_rate_deg_s = 350.0f,
     .acc_limit_deg_s2 = 1500.0f},
    /* Roll */
    {.axis_id = 3,
     .axis_sign = 1.0f,
     .servo_zero_deg = 0.0f,
     .pos_min_deg = -45.0f,
     .pos_max_deg = 45.0f,
     .pulse_min_us = 1000,
     .pulse_max_us = 2000,
     .slew_rate_deg_s = 300.0f,
     .acc_limit_deg_s2 = 1200.0f},
};

const axis_params_t *axis_params_get(axis_t axis) {
  if ((int)axis < 0 || axis > AXIS_ROLL) {
    return &k_axis_table[0];
  }
  return &k_axis_table[(int)axis];
}
