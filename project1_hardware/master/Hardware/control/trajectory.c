#include "trajectory.h"

#include "math_utils.h"

float rate_limit(float x, float x_prev, float rate, float dt) {
  return math_rate_limit(x, x_prev, rate, dt);
}
