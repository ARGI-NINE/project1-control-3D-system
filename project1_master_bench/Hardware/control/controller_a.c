#include "controller_a.h"

#include <stdbool.h>

#include "axis_params.h"
#include "math_utils.h"

typedef struct {
  float int_e_w;
  float d_hat;
  float pos_ref;
  bool initialized;
} ctrl_a_axis_state_t;

typedef struct {
  float k_theta;
  float wmax;
  float kp_w;
  float ki_w;
  float kd;
  float leak;
  float umax;
  float i_max;
  float d_max;
} ctrl_a_param_t;

static ctrl_a_axis_state_t g_axis_state[3];
static const ctrl_a_param_t k_param = {
    .k_theta = 7.0f,
    .wmax = 250.0f,
    .kp_w = 1.3f,
    .ki_w = 0.65f,
    .kd = 1.2f,
    .leak = 0.2f,
    .umax = 300.0f,
    .i_max = 250.0f,
    .d_max = 200.0f,
};

void ctrl_a_init(void) {
  for (int i = 0; i < 3; ++i) {
    g_axis_state[i] = (ctrl_a_axis_state_t){0};
  }
}

float ctrl_a_step(axis_t axis, float theta_deg, float omega_deg_s, float theta_ref_deg,
                  float dt) {
  if ((int)axis < 0 || axis > AXIS_ROLL || dt <= 0.0f) {
    return theta_deg;
  }
  ctrl_a_axis_state_t *st = &g_axis_state[(int)axis];
  const axis_params_t *cfg = axis_params_get(axis);

  if (!st->initialized) {
    st->initialized = true;
    st->pos_ref = math_clampf(theta_deg, cfg->pos_min_deg, cfg->pos_max_deg);
  }

  const float e_theta = math_wrap_deg(theta_ref_deg - theta_deg);
  const float omega_ref = math_clampf(k_param.k_theta * e_theta, -k_param.wmax, k_param.wmax);

  const float e_w = omega_ref - omega_deg_s;
  st->int_e_w = math_clampf(st->int_e_w + e_w * dt, -k_param.i_max, k_param.i_max);
  st->d_hat = math_clampf(st->d_hat + dt * (k_param.kd * e_w - k_param.leak * st->d_hat),
                          -k_param.d_max, k_param.d_max);

  float u = (k_param.kp_w * e_w + k_param.ki_w * st->int_e_w) - st->d_hat;
  u = math_clampf(u, -k_param.umax, k_param.umax);

  float pos_next = st->pos_ref + u * dt;
  pos_next = math_rate_limit(pos_next, st->pos_ref, cfg->slew_rate_deg_s, dt);
  pos_next = math_clampf(pos_next, cfg->pos_min_deg, cfg->pos_max_deg);
  st->pos_ref = pos_next;
  return st->pos_ref;
}
