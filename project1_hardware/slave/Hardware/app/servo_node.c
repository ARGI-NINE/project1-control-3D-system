#include "servo_node.h"

#include <stdbool.h>

#include "../common/include/math_utils.h"
#include "../comm/can_rx.h"
#include "../config/axis_config.h"
#include "../driver/servo_pwm.h"

typedef struct {
  slave_state_t state;
  slave_fault_t fault;
  float target_pos_deg;
  float cur_pos_deg;
  float cur_vel_deg_s;
  uint16_t seq_echo;
  float vbat_v;
} servo_node_ctx_t;

static servo_node_ctx_t g_ctx;

static float move_towards(float cur, float tgt, float max_step) {
  return math_move_towards(cur, tgt, max_step);
}

static uint16_t angle_to_pulse_us(float deg) {
  const slave_axis_config_t *cfg = slave_axis_config_get();
  float t = 0.0f;
  if (cfg->pos_max_deg > cfg->pos_min_deg) {
    t = (deg - cfg->pos_min_deg) / (cfg->pos_max_deg - cfg->pos_min_deg);
  }
  t = math_clampf(t, 0.0f, 1.0f);
  const float pulse = (float)cfg->pulse_min_us +
                      t * (float)(cfg->pulse_max_us - cfg->pulse_min_us);
  return (uint16_t)(pulse + 0.5f);
}

static void publish_status(const slave_axis_config_t *cfg) {
  if (!cfg) {
    return;
  }

  can_status_t st = {0};
  st.seq_echo = g_ctx.seq_echo;
  st.pulse_us = servo_pwm_get_pulse_us();
  st.cur_pos_cdeg = math_deg_to_cdeg_sat(g_ctx.cur_pos_deg);
  st.fault = (uint8_t)g_ctx.fault;
  st.vbat_01v = (uint8_t)math_clampf(g_ctx.vbat_v * 10.0f, 0.0f, 255.0f);
  (void)can_tx_status(cfg->axis_id, &st);
}

void servo_node_set_time_ms(uint32_t now_ms) {
  can_rx_set_time_ms(now_ms);
}

void servo_node_update_vbat(float vbat_v) { g_ctx.vbat_v = vbat_v; }

void servo_node_init(void) {
  const slave_axis_config_t *cfg = slave_axis_config_get();
  servo_pwm_init(cfg->pwm_hz);
  can_rx_init();

  g_ctx = (servo_node_ctx_t){0};
  g_ctx.state = SLAVE_INIT;
  g_ctx.fault = SLAVE_FAULT_OK;
  g_ctx.cur_pos_deg = cfg->servo_zero_deg;
  g_ctx.target_pos_deg = cfg->servo_zero_deg;
  g_ctx.vbat_v = 6.0f;

  servo_pwm_set_pulse_us(angle_to_pulse_us(g_ctx.cur_pos_deg));
  publish_status(cfg);
}

void servo_node_on_timeout(void) {
  const slave_axis_config_t *cfg = slave_axis_config_get();
  g_ctx.state = SLAVE_TIMEOUT;
  g_ctx.fault = SLAVE_FAULT_TIMEOUT;
  if (cfg->return_to_center_on_timeout) {
    g_ctx.target_pos_deg = cfg->servo_zero_deg;
  }
}

void servo_node_step_200hz(float dt_s) {
  const slave_axis_config_t *cfg = slave_axis_config_get();
  float dt = dt_s;
  if (dt < SLAVE_CTRL_DT_MIN_S) {
    dt = SLAVE_CTRL_DT_MIN_S;
  } else if (dt > SLAVE_CTRL_DT_MAX_S) {
    dt = SLAVE_CTRL_DT_MAX_S;
  }

  if (g_ctx.vbat_v < cfg->vbat_low_v) {
    g_ctx.state = SLAVE_FAULT;
    g_ctx.fault = SLAVE_FAULT_VBAT_LOW;
    g_ctx.target_pos_deg = cfg->servo_zero_deg;
  }

  int16_t target_cdeg = 0;
  uint16_t seq = 0;
  bool has_cmd = can_get_latest_target(&target_cdeg, &seq);
  const bool link_timeout = has_cmd && (can_rx_ms_since_last_msg() > cfg->timeout_ms);

  if (has_cmd && !link_timeout) {
    float cmd_deg = math_cdeg_to_deg(target_cdeg);
    cmd_deg = cfg->servo_zero_deg + cfg->axis_sign * cmd_deg;
    g_ctx.target_pos_deg = math_clampf(cmd_deg, cfg->pos_min_deg, cfg->pos_max_deg);
    g_ctx.seq_echo = seq;
    if (g_ctx.state != SLAVE_FAULT) {
      g_ctx.state = SLAVE_RUN;
      g_ctx.fault = SLAVE_FAULT_OK;
    }
  }

  if (link_timeout && g_ctx.state != SLAVE_FAULT) {
    servo_node_on_timeout();
  }

  if (g_ctx.state == SLAVE_TIMEOUT && cfg->return_to_center_on_timeout) {
    const float back_slew = (cfg->timeout_return_time_s > 1.0e-4f)
                                ? (cfg->pos_max_deg - cfg->pos_min_deg) /
                                      cfg->timeout_return_time_s
                                : cfg->slew_rate_deg_s;
    g_ctx.target_pos_deg = move_towards(g_ctx.target_pos_deg, cfg->servo_zero_deg,
                                        back_slew * dt);
  }

  float vel_cmd = 4.0f * (g_ctx.target_pos_deg - g_ctx.cur_pos_deg);
  vel_cmd = math_clampf(vel_cmd, -cfg->slew_rate_deg_s, cfg->slew_rate_deg_s);
  g_ctx.cur_vel_deg_s = move_towards(g_ctx.cur_vel_deg_s, vel_cmd, cfg->acc_limit_deg_s2 * dt);
  g_ctx.cur_pos_deg += g_ctx.cur_vel_deg_s * dt;
  g_ctx.cur_pos_deg = math_clampf(g_ctx.cur_pos_deg, cfg->pos_min_deg, cfg->pos_max_deg);

  if (g_ctx.cur_pos_deg <= cfg->pos_min_deg + 0.01f ||
      g_ctx.cur_pos_deg >= cfg->pos_max_deg - 0.01f) {
    if (g_ctx.state == SLAVE_RUN) {
      g_ctx.fault = SLAVE_FAULT_LIMIT;
    }
  } else if (g_ctx.state == SLAVE_RUN) {
    g_ctx.fault = SLAVE_FAULT_OK;
  }

  servo_pwm_set_pulse_us(angle_to_pulse_us(g_ctx.cur_pos_deg));
  publish_status(cfg);
}

void servo_node_get_status(servo_node_status_t *out) {
  if (!out) {
    return;
  }
  out->seq_echo = g_ctx.seq_echo;
  out->pulse_us = servo_pwm_get_pulse_us();
  out->cur_pos_cdeg = math_deg_to_cdeg_sat(g_ctx.cur_pos_deg);
  out->fault = (uint8_t)g_ctx.fault;
  out->vbat_01v = (uint8_t)math_clampf(g_ctx.vbat_v * 10.0f, 0.0f, 255.0f);
  out->state = g_ctx.state;
}
