#include "axis_map.h"

float axis_extract_rate_deg_s(const imu_sample_t *s, axis_t axis) {
  if (!s) {
    return 0.0f;
  }
  switch (axis) {
  case AXIS_YAW:
    return s->gz;
  case AXIS_PITCH:
    return s->gy;
  case AXIS_ROLL:
  default:
    return s->gx;
  }
}
