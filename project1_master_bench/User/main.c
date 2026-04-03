#include <stdbool.h>
#include <stdint.h>

#include "master_tick.h"
#include "Timer.h"

#include "master_node.h"
#include "mpu6050.h"
#include "scheduler.h"
#include "can_proto.h"
#include "project_config.h"
#include "perf_diag.h"
#if MASTER_DIAG_ENABLE_OLED
#include "oled_diag.h"
#endif

static volatile uint32_t g_ms_ticks = 0u;
static volatile uint32_t g_tick_count = 0u;
static volatile uint32_t g_tick_us_accum = 0u;
static imu_sample_t g_last_imu = {0};
static imu_sample_t g_prev_polled_imu = {0};
static bool g_prev_polled_imu_valid = false;
static bool g_has_imu = false;
static scheduler_counters_t g_sched_dbg = {0};
static can_proto_stats_t g_can_dbg = {0};
static uint32_t g_last_ctrl_tick = 0u;
static bool g_ctrl_time_inited = false;
#if MASTER_DIAG_ENABLE_OLED
static uint8_t g_oled_refresh_div = 0u;
#endif

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

static bool imu_sample_equal(const imu_sample_t *lhs, const imu_sample_t *rhs) {
  if (!lhs || !rhs) {
    return false;
  }
  return (lhs->ax == rhs->ax) && (lhs->ay == rhs->ay) && (lhs->az == rhs->az) &&
         (lhs->gx == rhs->gx) && (lhs->gy == rhs->gy) && (lhs->gz == rhs->gz);
}

static void master_task_400hz(void) {
  perf_diag_hook_begin(PERF_HOOK_400HZ);

  const float dt_s = master_compute_ctrl_dt_s();
  imu_sample_t imu = {0};
  imu_sample_t node_imu = {0};
  bool node_imu_valid = false;
  bool imu_same_as_prev = false;
  const bool imu_ok = mpu6050_poll_sample_dma(&imu);
  if (imu_ok) {
    if (g_prev_polled_imu_valid) {
      imu_same_as_prev = imu_sample_equal(&imu, &g_prev_polled_imu);
    }
    g_prev_polled_imu = imu;
    g_prev_polled_imu_valid = true;

    g_last_imu = imu;
    g_has_imu = true;
    node_imu = imu;
    node_imu_valid = true;
  }
  perf_diag_set_imu_repeat(imu_ok, imu_same_as_prev);

  if (!imu_ok) {
    const master_state_t state = master_node_get_state();
    if (state == MASTER_RUN && g_has_imu) {
      node_imu = g_last_imu;
      node_imu_valid = true;
    }
  }
  master_node_step_400hz(&node_imu, node_imu_valid, dt_s);

  (void)mpu6050_start_read_dma();
  perf_diag_hook_end(PERF_HOOK_400HZ);
}

static void master_task_can_200hz(void) {
  perf_diag_hook_begin(PERF_HOOK_200HZ);
  master_node_step_can_200hz();
  can_proto_poll_status();
  perf_diag_hook_end(PERF_HOOK_200HZ);
}

static void master_task_log_1hz(void) {
  perf_diag_hook_begin(PERF_HOOK_1HZ);

  scheduler_get_counters(&g_sched_dbg);
  can_proto_get_stats(&g_can_dbg);
  perf_diag_set_scheduler(&g_sched_dbg);
  perf_diag_set_can_stats(&g_can_dbg);

  for (uint8_t axis = 1u; axis <= 3u; ++axis) {
    can_status_t st = {0};
    uint32_t age_ms = 0u;
    const bool valid = can_proto_get_latest_status(axis, &st, &age_ms);
    (void)st;
    perf_diag_set_axis_status(axis, valid, age_ms);
  }

#if MASTER_DIAG_ENABLE_OLED
  if (++g_oled_refresh_div >= MASTER_DIAG_OLED_REFRESH_DIV) {
    g_oled_refresh_div = 0u;
    oled_diag_render();
  }
#endif

  perf_diag_hook_end(PERF_HOOK_1HZ);
}

int main(void) {
#if MASTER_DIAG_ENABLE_OLED
  oled_diag_init();
#endif

  master_node_init();
  master_node_select_controller(false);
  master_node_set_target_deg(0.0f, 0.0f, 0.0f);

  (void)mpu6050_init();
  (void)mpu6050_start_read_dma();

  perf_diag_init();

  scheduler_init();
  scheduler_register_hooks(master_task_400hz, master_task_can_200hz, master_task_log_1hz);

#if MASTER_DIAG_ENABLE_OLED
  /* Draw one frame before timer IRQ starts to avoid startup overrun pollution. */
  oled_diag_render();
  g_oled_refresh_div = 0u;
#endif

  Timer_Init();

  while (1) {
    app_loop();
  }
}
