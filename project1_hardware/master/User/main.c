#include <stdbool.h>
#include <stdint.h>

#include "master_tick.h"
#include "Timer.h"

#include "master_node.h"
#include "mpu6050.h"
#include "scheduler.h"
#include "can_proto.h"
#include "project_config.h"

static volatile uint32_t g_ms_ticks = 0u;
static volatile uint32_t g_tick_count = 0u;
static volatile uint32_t g_tick_us_accum = 0u;
static imu_sample_t g_last_imu = {0};
static bool g_has_imu = false;
static uint32_t g_last_ctrl_tick = 0u;
static bool g_ctrl_time_inited = false;

void master_tick_isr(void) {
  const uint32_t prev_ms = g_ms_ticks;
  ++g_tick_count;
  g_tick_us_accum += MASTER_TICK_PERIOD_US;
  while (g_tick_us_accum >= 1000u) {
    g_tick_us_accum -= 1000u;
    ++g_ms_ticks;
  }
  if (g_ms_ticks != prev_ms) {
    can_proto_set_time_ms(g_ms_ticks);
  }
  scheduler_tick_isr();
}

static float master_compute_ctrl_dt_s(void) {
  const uint32_t now_tick = g_tick_count;
  if (!g_ctrl_time_inited) {
    g_ctrl_time_inited = true;
    g_last_ctrl_tick = now_tick;
    return MASTER_CTRL_DT_S;
  }

  uint32_t delta_tick = now_tick - g_last_ctrl_tick;
  g_last_ctrl_tick = now_tick;
  if (delta_tick == 0u) {
    delta_tick = 1u;
  }

  float dt_s = (float)delta_tick / (float)MASTER_TICK_HZ;
  if (dt_s < MASTER_CTRL_DT_MIN_S) {
    dt_s = MASTER_CTRL_DT_MIN_S;
  } else if (dt_s > MASTER_CTRL_DT_MAX_S) {
    dt_s = MASTER_CTRL_DT_MAX_S;
  }
  return dt_s;
}

static void master_task_400hz(void) {
  const float dt_s = master_compute_ctrl_dt_s();
  imu_sample_t imu = {0};
  imu_sample_t node_imu = {0};
  bool node_imu_valid = false;
  const bool imu_ok = mpu6050_poll_sample_dma(&imu);
  if (imu_ok) {
    g_last_imu = imu;
    g_has_imu = true;
    node_imu = imu;
    node_imu_valid = true;
  }

  if (!imu_ok) {
    const master_state_t state = master_node_get_state();
    if (state == MASTER_RUN && g_has_imu) {
      node_imu = g_last_imu;
      node_imu_valid = true;
    }
  }
  master_node_step_400hz(&node_imu, node_imu_valid, dt_s);

  (void)mpu6050_start_read_dma();
}

static void master_task_can_200hz(void) {
  master_node_step_can_200hz();
  can_proto_poll_status();
}

int main(void) {
  master_node_init();
  master_node_select_controller(false);
  master_node_set_target_deg(0.0f, 0.0f, 0.0f);

  (void)mpu6050_init();
  (void)mpu6050_start_read_dma();

  scheduler_init();
  scheduler_register_hooks(master_task_400hz, master_task_can_200hz);

  Timer_Init();

  while (1) {
    app_loop();
  }
}
