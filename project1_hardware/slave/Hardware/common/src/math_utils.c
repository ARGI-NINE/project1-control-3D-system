#include "../include/math_utils.h"

#include <math.h>

static const float k_pi = 3.14159265358979323846f;

float math_clampf(float x, float lo, float hi) {
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

float math_wrap_deg(float a_deg) {
  while (a_deg > 180.0f) {
    a_deg -= 360.0f;
  }
  while (a_deg < -180.0f) {
    a_deg += 360.0f;
  }
  return a_deg;
}

float math_rate_limit(float x, float x_prev, float rate, float dt) {
  const float max_step = rate * dt;
  if (x > x_prev + max_step) {
    return x_prev + max_step;
  }
  if (x < x_prev - max_step) {
    return x_prev - max_step;
  }
  return x;
}

float math_move_towards(float cur, float tgt, float max_step) {
  if (tgt > cur + max_step) {
    return cur + max_step;
  }
  if (tgt < cur - max_step) {
    return cur - max_step;
  }
  return tgt;
}

float math_deg_to_rad(float deg) { return deg * k_pi / 180.0f; }

float math_rad_to_deg(float rad) { return rad * 180.0f / k_pi; }

int16_t math_deg_to_cdeg_sat(float deg) {
  float cdeg = deg * 100.0f;
  if (cdeg > 32767.0f) {
    cdeg = 32767.0f;
  } else if (cdeg < -32768.0f) {
    cdeg = -32768.0f;
  }
  return (int16_t)cdeg;
}

float math_cdeg_to_deg(int16_t cdeg) { return 0.01f * (float)cdeg; }
