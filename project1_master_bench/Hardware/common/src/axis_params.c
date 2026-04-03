#include "axis_params.h"

static const axis_params_t k_axis_table[3] = {
    {.pos_min_deg = -90.0f, .pos_max_deg = 90.0f, .slew_rate_deg_s = 400.0f},
    {.pos_min_deg = -60.0f, .pos_max_deg = 60.0f, .slew_rate_deg_s = 350.0f},
    {.pos_min_deg = -45.0f, .pos_max_deg = 45.0f, .slew_rate_deg_s = 300.0f},
};

const axis_params_t *axis_params_get(axis_t axis) {
  if ((int)axis < 0 || axis > AXIS_ROLL) {
    return &k_axis_table[0];
  }
  return &k_axis_table[(int)axis];
}
