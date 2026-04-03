#ifndef __AXIS_PARAMS_H
#define __AXIS_PARAMS_H
#include "types.h"

typedef struct {
  float pos_min_deg;    /* mechanical limit */
  float pos_max_deg;    /* mechanical limit */
  float slew_rate_deg_s; /* command rate limit */
} axis_params_t;

const axis_params_t *axis_params_get(axis_t axis);

#endif /* __AXIS_PARAMS_H */

