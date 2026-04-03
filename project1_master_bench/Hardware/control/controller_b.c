#include "controller_b.h"

#include <stdbool.h>
#include <math.h>

#include "axis_params.h"
#include "math_utils.h"

typedef struct {
  float e_int;
  float k;
  float s;
  float pos_ref;
  float last_u;
  bool initialized;
} ctrl_b_axis_state_t;

typedef struct {
  float lambda;
  float ki;
  float e_max;
  float phi;
  float kmin;
  float kmax;
  float gamma;
  float s0;
  float umax;
} ctrl_b_param_t;

static ctrl_b_axis_state_t g_axis_state[3];
static const ctrl_b_param_t k_param = {
    .lambda = 6.0f,
    .ki = 2.0f,
    .e_max = 90.0f,
    .phi = 12.0f,
    .kmin = 20.0f,
    .kmax = 120.0f,
    .gamma = 60.0f,
    .s0 = 2.5f,
    .umax = 300.0f,
};

void ctrl_b_init(void) {
  for (int i = 0; i < 3; ++i) {
    g_axis_state[i] = (ctrl_b_axis_state_t){0};
    g_axis_state[i].k = k_param.kmin;
  }
}

float ctrl_b_step(axis_t axis, float theta_deg, float omega_deg_s, float theta_ref_deg,
                  float dt) {
  if ((int)axis < 0 || axis > AXIS_ROLL || dt <= 0.0f) {
    return theta_deg;
  }
  ctrl_b_axis_state_t *st = &g_axis_state[(int)axis];
  const axis_params_t *cfg = axis_params_get(axis);

  if (!st->initialized) {
    st->initialized = true;
    st->pos_ref = math_clampf(theta_deg, cfg->pos_min_deg, cfg->pos_max_deg);
  }

  const float e = math_wrap_deg(theta_ref_deg - theta_deg);
  st->e_int = math_clampf(st->e_int + e * dt, -k_param.e_max, k_param.e_max);
  st->s = -omega_deg_s + k_param.lambda * e + k_param.ki * st->e_int;

  st->k = math_clampf(st->k + dt * k_param.gamma * (fabsf(st->s) - k_param.s0),
                      k_param.kmin, k_param.kmax);

  const float sat = math_clampf(st->s / k_param.phi, -1.0f, 1.0f);
  const float u_raw = k_param.lambda * e + k_param.ki * st->e_int + st->k * sat;
  float u = 0.8f * st->last_u + 0.2f * u_raw;
  u = math_clampf(u, -k_param.umax, k_param.umax);
  st->last_u = u;

  float pos_next = st->pos_ref + u * dt;
  pos_next = math_rate_limit(pos_next, st->pos_ref, cfg->slew_rate_deg_s, dt);
  pos_next = math_clampf(pos_next, cfg->pos_min_deg, cfg->pos_max_deg);
  st->pos_ref = pos_next;
  return st->pos_ref;
}
