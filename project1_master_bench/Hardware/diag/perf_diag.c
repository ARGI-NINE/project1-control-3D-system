#include "perf_diag.h"

#include "Timer.h"

typedef struct {
  uint32_t last_us;
  uint32_t max_us;
  uint32_t start_us;
  uint32_t sample_cnt;
  uint32_t sum_us;
} hook_slot_t;

static hook_slot_t g_slot[PERF_HOOK_COUNT] = {0};
static uint32_t g_over_budget_400hz = 0u;
static uint32_t g_imu_valid_count = 0u;
static uint32_t g_imu_repeat_count = 0u;
static uint8_t g_imu_repeat_last = 0u;
static uint8_t g_axis_timeout_mask = 0u;
static uint8_t g_axis_missing_mask = 0u;
static scheduler_counters_t g_sched = {0};
static can_proto_stats_t g_can = {0};

static uint8_t perf_diag_make_timing_fail_mask(void) {
  uint8_t fail_mask = 0u;
  if (g_over_budget_400hz > 0u) {
    fail_mask |= PERF_TIMING_FAIL_400HZ_BUDGET;
  }
  if (g_sched.overrun_400hz > 0u) {
    fail_mask |= PERF_TIMING_FAIL_400HZ_OVERRUN;
  }
  if (g_sched.overrun_can_200hz > 0u) {
    fail_mask |= PERF_TIMING_FAIL_200HZ_OVERRUN;
  }
  if (g_sched.pending_400hz > 0u) {
    fail_mask |= PERF_TIMING_FAIL_400HZ_PENDING;
  }
  if (g_sched.pending_can_200hz > 0u) {
    fail_mask |= PERF_TIMING_FAIL_200HZ_PENDING;
  }
  return fail_mask;
}

static uint32_t hook_budget_us(perf_hook_t hook) {
  if (hook == PERF_HOOK_400HZ) {
    return MASTER_DIAG_400HZ_BUDGET_US;
  }
  return 0u;
}

void perf_diag_init(void) {
  for (uint8_t i = 0u; i < PERF_HOOK_COUNT; ++i) {
    g_slot[i] = (hook_slot_t){0};
  }
  g_over_budget_400hz = 0u;
  g_imu_valid_count = 0u;
  g_imu_repeat_count = 0u;
  g_imu_repeat_last = 0u;
  g_axis_timeout_mask = 0u;
  g_axis_missing_mask = 0u;
  g_sched = (scheduler_counters_t){0};
  g_can = (can_proto_stats_t){0};
}

void perf_diag_hook_begin(perf_hook_t hook) {
  if ((uint32_t)hook >= (uint32_t)PERF_HOOK_COUNT) {
    return;
  }
  g_slot[(uint32_t)hook].start_us = Timer_NowUs();
}

void perf_diag_hook_end(perf_hook_t hook) {
  if ((uint32_t)hook >= (uint32_t)PERF_HOOK_COUNT) {
    return;
  }

  hook_slot_t *slot = &g_slot[(uint32_t)hook];
  const uint32_t elapsed = Timer_NowUs() - slot->start_us;
  slot->last_us = elapsed;
  if (elapsed > slot->max_us) {
    slot->max_us = elapsed;
  }
  slot->sum_us += elapsed;
  slot->sample_cnt += 1u;

  if (hook == PERF_HOOK_400HZ) {
    if (elapsed > hook_budget_us(hook)) {
      ++g_over_budget_400hz;
    }
  }
}

void perf_diag_set_scheduler(const scheduler_counters_t *cnt) {
  if (!cnt) {
    return;
  }
  g_sched = *cnt;
}

void perf_diag_set_can_stats(const can_proto_stats_t *stats) {
  if (!stats) {
    return;
  }
  g_can = *stats;
}

void perf_diag_set_axis_status(uint8_t axis_id, bool valid, uint32_t age_ms) {
  if (axis_id < 1u || axis_id > 3u) {
    return;
  }
  const uint8_t bit = (uint8_t)(1u << (axis_id - 1u));

  if (!valid) {
    g_axis_missing_mask |= bit;
    g_axis_timeout_mask &= (uint8_t)(~bit);
    return;
  }

  g_axis_missing_mask &= (uint8_t)(~bit);
  if (age_ms > MASTER_DIAG_STATUS_TIMEOUT_MS) {
    g_axis_timeout_mask |= bit;
  } else {
    g_axis_timeout_mask &= (uint8_t)(~bit);
  }
}

void perf_diag_set_imu_repeat(bool sample_valid, bool same_as_prev) {
  g_imu_repeat_last = 0u;
  if (!sample_valid) {
    return;
  }

  ++g_imu_valid_count;
  if (same_as_prev) {
    ++g_imu_repeat_count;
    g_imu_repeat_last = 1u;
  }
}

void perf_diag_get_snapshot(perf_snapshot_t *out) {
  if (!out) {
    return;
  }

  out->last_us_400hz = g_slot[PERF_HOOK_400HZ].last_us;
  out->max_us_400hz = g_slot[PERF_HOOK_400HZ].max_us;
  out->last_us_200hz = g_slot[PERF_HOOK_200HZ].last_us;
  out->max_us_200hz = g_slot[PERF_HOOK_200HZ].max_us;
  out->last_us_1hz = g_slot[PERF_HOOK_1HZ].last_us;
  out->max_us_1hz = g_slot[PERF_HOOK_1HZ].max_us;

  out->over_budget_400hz = g_over_budget_400hz;
  out->overrun_400hz = g_sched.overrun_400hz;
  out->overrun_can_200hz = g_sched.overrun_can_200hz;
  out->overrun_log_1hz = 0u;
  out->pending_400hz = g_sched.pending_400hz;
  out->pending_can_200hz = g_sched.pending_can_200hz;
  out->pending_log_1hz = 0u;

  out->can_status_rx_ok = g_can.status_rx_ok;
  out->can_status_rx_bad = g_can.status_rx_bad_id_or_dlc + g_can.status_rx_unpack_fail;
  out->imu_valid_count = g_imu_valid_count;
  out->imu_repeat_count = g_imu_repeat_count;
  out->imu_repeat_last = g_imu_repeat_last;
  out->axis_timeout_mask = g_axis_timeout_mask;
  out->axis_missing_mask = g_axis_missing_mask;
  out->timing_fail_mask = perf_diag_make_timing_fail_mask();
  out->timing_passed = (uint8_t)(out->timing_fail_mask == 0u ? 1u : 0u);
}

void perf_diag_get_timing_result(uint8_t *out_fail_mask, bool *out_passed) {
  const uint8_t fail_mask = perf_diag_make_timing_fail_mask();
  if (out_fail_mask) {
    *out_fail_mask = fail_mask;
  }
  if (out_passed) {
    *out_passed = (fail_mask == 0u);
  }
}
