#include "mahony.h"

#include <math.h>

#include "math_utils.h"

typedef struct {
  float kp;
  float ki;
  float ex_int;
  float ey_int;
  float ez_int;
} mahony_ctx_t;

static mahony_ctx_t g_ctx = {.kp = 1.2f, .ki = 0.02f};
static const float k_pi = 3.14159265358979323846f;

static float inv_sqrt(float x) { return 1.0f / sqrtf(x); }

void mahony_init(float kp, float ki) {
  g_ctx.kp = kp;
  g_ctx.ki = ki;
  g_ctx.ex_int = 0.0f;
  g_ctx.ey_int = 0.0f;
  g_ctx.ez_int = 0.0f;
}

void mahony_update(const imu_sample_t *s, float dt, quat_t *q_inout) {
  if (!s || !q_inout || dt <= 0.0f) {
    return;
  }

  float q0 = q_inout->w;
  float q1 = q_inout->x;
  float q2 = q_inout->y;
  float q3 = q_inout->z;

  float ax = s->ax;
  float ay = s->ay;
  float az = s->az;
  float gx = math_deg_to_rad(s->gx);
  float gy = math_deg_to_rad(s->gy);
  float gz = math_deg_to_rad(s->gz);

  const float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
  if (acc_norm > 1.0e-6f) {
    const float inv_norm = inv_sqrt(acc_norm * acc_norm);
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    float acc_weight = 1.0f;
    {
      float g_dev = fabsf(acc_norm - 1.0f);
      if (g_dev > 0.05f) {
        acc_weight = math_clampf(1.0f - 2.5f * g_dev, 0.05f, 1.0f);
      }
    }

    ex *= acc_weight;
    ey *= acc_weight;
    ez *= acc_weight;

    if (g_ctx.ki > 0.0f) {
      g_ctx.ex_int += ex * dt;
      g_ctx.ey_int += ey * dt;
      g_ctx.ez_int += ez * dt;
      gx += g_ctx.ki * g_ctx.ex_int;
      gy += g_ctx.ki * g_ctx.ey_int;
      gz += g_ctx.ki * g_ctx.ez_int;
    }

    gx += g_ctx.kp * ex;
    gy += g_ctx.kp * ey;
    gz += g_ctx.kp * ez;
  }

  float half_dt = 0.5f * dt;
  float qa = q0;
  float qb = q1;
  float qc = q2;

  q0 += (-qb * gx - qc * gy - q3 * gz) * half_dt;
  q1 += (qa * gx + qc * gz - q3 * gy) * half_dt;
  q2 += (qa * gy - qb * gz + q3 * gx) * half_dt;
  q3 += (qa * gz + qb * gy - qc * gx) * half_dt;

  float q_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q_inout->w = q0 * q_norm;
  q_inout->x = q1 * q_norm;
  q_inout->y = q2 * q_norm;
  q_inout->z = q3 * q_norm;
}

void quat_to_euler(const quat_t *q, float *roll_deg, float *pitch_deg, float *yaw_deg) {
  if (!q) {
    return;
  }
  const float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
  const float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
  const float roll = atan2f(sinr_cosp, cosr_cosp);

  const float sinp = 2.0f * (q->w * q->y - q->z * q->x);
  float pitch = 0.0f;
  if (fabsf(sinp) >= 1.0f) {
    pitch = copysignf(k_pi / 2.0f, sinp);
  } else {
    pitch = asinf(sinp);
  }

  const float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
  const float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
  const float yaw = atan2f(siny_cosp, cosy_cosp);

  if (roll_deg) {
    *roll_deg = math_rad_to_deg(roll);
  }
  if (pitch_deg) {
    *pitch_deg = math_rad_to_deg(pitch);
  }
  if (yaw_deg) {
    *yaw_deg = math_rad_to_deg(yaw);
  }
}
