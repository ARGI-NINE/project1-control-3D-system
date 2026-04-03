#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>

float math_clampf(float x, float lo, float hi);
float math_wrap_deg(float a_deg);
float math_rate_limit(float x, float x_prev, float rate, float dt);
float math_move_towards(float cur, float tgt, float max_step);
float math_deg_to_rad(float deg);
float math_rad_to_deg(float rad);
int16_t math_deg_to_cdeg_sat(float deg);
float math_cdeg_to_deg(int16_t cdeg);

#endif /* MATH_UTILS_H */

