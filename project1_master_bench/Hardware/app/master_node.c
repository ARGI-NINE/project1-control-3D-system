#include "master_node.h"

#include <string.h>

#include "axis_map.h"
#include "can_proto.h"
#include "controller_a.h"
#include "controller_b.h"
#include "mahony.h"
#include "mpu6050.h"
#include "math_utils.h"
#include "trajectory.h"

typedef struct {
  master_state_t state;
  bool use_algo_b;
  quat_t q;
  imu_sample_t bias;
  float ref_deg[3];
  float pos_ref_deg[3];
  float calib_s;
  float calib_sum_gx;
  float calib_sum_gy;
  float calib_sum_gz;
  uint32_t calib_samples;
  uint16_t seq;
} master_node_ctx_t;

static master_node_ctx_t g_ctx;

void master_node_init(void) {
  (void)memset(&g_ctx, 0, sizeof(g_ctx));
  g_ctx.state = MASTER_INIT;
  g_ctx.q = (quat_t){.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
  can_proto_init();
  ctrl_a_init();
  ctrl_b_init();
  mahony_init(1.5f, 0.03f);
}

void master_node_select_controller(bool use_algo_b) { g_ctx.use_algo_b = use_algo_b; }

void master_node_set_target_deg(float yaw_deg, float pitch_deg, float roll_deg) {
  g_ctx.ref_deg[AXIS_YAW] = yaw_deg;
  g_ctx.ref_deg[AXIS_PITCH] = pitch_deg;
  g_ctx.ref_deg[AXIS_ROLL] = roll_deg;
}

static void run_control(const imu_sample_t *imu, bool imu_valid, float dt) {
  if (!imu_valid || !imu) {
    g_ctx.state = MASTER_SAFE;
    return;
  }
  mahony_update(imu, dt, &g_ctx.q);

  float roll_deg = 0.0f;
  float pitch_deg = 0.0f;
  float yaw_deg = 0.0f;
  quat_to_euler(&g_ctx.q, &roll_deg, &pitch_deg, &yaw_deg);
  const float theta_deg[3] = {yaw_deg, pitch_deg, roll_deg};

  for (int i = 0; i < 3; ++i) {
    axis_t axis = (axis_t)i;
    float theta = theta_deg[i];
    float omega = axis_extract_rate_deg_s(imu, axis);
    float pos_ref = 0.0f;
    if (g_ctx.use_algo_b) {
      pos_ref = ctrl_b_step(axis, theta, omega, g_ctx.ref_deg[i], dt);
    } else {
      pos_ref = ctrl_a_step(axis, theta, omega, g_ctx.ref_deg[i], dt);
    }
    g_ctx.pos_ref_deg[i] = pos_ref;
  }
}

void master_node_step_400hz(const imu_sample_t *imu, bool imu_valid, float dt) {
  switch (g_ctx.state) {
  case MASTER_INIT:
    if (!mpu6050_is_ready()) {
      if (!mpu6050_init()) {
        g_ctx.state = MASTER_SAFE;
        break;
      }
    }
    g_ctx.bias = (imu_sample_t){0};
    mpu6050_set_bias(&g_ctx.bias);
    g_ctx.calib_s = 0.0f;
    g_ctx.calib_sum_gx = 0.0f;
    g_ctx.calib_sum_gy = 0.0f;
    g_ctx.calib_sum_gz = 0.0f;
    g_ctx.calib_samples = 0u;
    g_ctx.state = MASTER_CALIB;
    break;

  case MASTER_CALIB:
    if (imu_valid && imu) {
      g_ctx.calib_sum_gx += imu->gx;
      g_ctx.calib_sum_gy += imu->gy;
      g_ctx.calib_sum_gz += imu->gz;
      ++g_ctx.calib_samples;
    }
    g_ctx.calib_s += dt;
    if (g_ctx.calib_s >= 2.0f) {
      if (g_ctx.calib_samples == 0u) {
        g_ctx.state = MASTER_SAFE;
        break;
      }
      g_ctx.bias = (imu_sample_t){0};
      g_ctx.bias.gx = g_ctx.calib_sum_gx / (float)g_ctx.calib_samples;
      g_ctx.bias.gy = g_ctx.calib_sum_gy / (float)g_ctx.calib_samples;
      g_ctx.bias.gz = g_ctx.calib_sum_gz / (float)g_ctx.calib_samples;
      mpu6050_set_bias(&g_ctx.bias);
      g_ctx.state = MASTER_RUN;
    }
    break;

  case MASTER_RUN:
    run_control(imu, imu_valid, dt);
    break;

  case MASTER_SAFE:
  default:
    g_ctx.pos_ref_deg[AXIS_YAW] = rate_limit(g_ctx.pos_ref_deg[AXIS_YAW], 0.0f, 120.0f, dt);
    g_ctx.pos_ref_deg[AXIS_PITCH] = rate_limit(g_ctx.pos_ref_deg[AXIS_PITCH], 0.0f, 120.0f, dt);
    g_ctx.pos_ref_deg[AXIS_ROLL] = rate_limit(g_ctx.pos_ref_deg[AXIS_ROLL], 0.0f, 120.0f, dt);
    break;
  }
}

void master_node_step_can_200hz(void) {
  if (g_ctx.state == MASTER_SAFE) {
    (void)can_send_cmd(g_ctx.seq++, 0, 0, 0);
    return;
  }

  (void)can_send_cmd(g_ctx.seq++, math_deg_to_cdeg_sat(g_ctx.pos_ref_deg[AXIS_YAW]),
                     math_deg_to_cdeg_sat(g_ctx.pos_ref_deg[AXIS_PITCH]),
                     math_deg_to_cdeg_sat(g_ctx.pos_ref_deg[AXIS_ROLL]));
}

master_state_t master_node_get_state(void) { return g_ctx.state; }

void master_node_get_pos_ref_deg(float *yaw_deg, float *pitch_deg, float *roll_deg) {
  if (yaw_deg) {
    *yaw_deg = g_ctx.pos_ref_deg[AXIS_YAW];
  }
  if (pitch_deg) {
    *pitch_deg = g_ctx.pos_ref_deg[AXIS_PITCH];
  }
  if (roll_deg) {
    *roll_deg = g_ctx.pos_ref_deg[AXIS_ROLL];
  }
}
