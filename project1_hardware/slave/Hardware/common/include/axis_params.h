#ifndef AXIS_PARAMS_H
#define AXIS_PARAMS_H

#include <stdint.h>

#include "types.h"

typedef struct {
  uint8_t axis_id;              /* 1/2/3 */
  float axis_sign;              /* +1/-1 */
  float servo_zero_deg;         /* middle offset */
  float pos_min_deg;            /* mechanical limit */
  float pos_max_deg;            /* mechanical limit */
  uint16_t pulse_min_us;        /* servo pulse min */
  uint16_t pulse_max_us;        /* servo pulse max */
  float slew_rate_deg_s;        /* command rate limit */
  float acc_limit_deg_s2;       /* optional accel limit */
} axis_params_t;

const axis_params_t *axis_params_get(axis_t axis);

#endif /* AXIS_PARAMS_H */

