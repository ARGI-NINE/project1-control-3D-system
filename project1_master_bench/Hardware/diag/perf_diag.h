#ifndef __MASTER_PERF_DIAG_H
#define __MASTER_PERF_DIAG_H
#include <stdbool.h>
#include <stdint.h>

#include "scheduler.h"
#include "can_proto.h"
#include "project_config.h"

typedef enum {
  PERF_HOOK_400HZ = 0,
  PERF_HOOK_200HZ = 1,
  PERF_HOOK_1HZ = 2,
  PERF_HOOK_COUNT
} perf_hook_t;

typedef struct {
  uint32_t last_us_400hz;
  uint32_t max_us_400hz;
  uint32_t last_us_200hz;
  uint32_t max_us_200hz;
  uint32_t last_us_1hz;
  uint32_t max_us_1hz;
  uint32_t over_budget_400hz;
  uint32_t overrun_400hz;
  uint32_t overrun_can_200hz;
  uint32_t overrun_log_1hz;
  uint16_t pending_400hz;
  uint16_t pending_can_200hz;
  uint16_t pending_log_1hz;
  uint32_t can_status_rx_ok;
  uint32_t can_status_rx_bad;
  uint32_t imu_valid_count;
  uint32_t imu_repeat_count;
  uint8_t imu_repeat_last;
  uint8_t axis_timeout_mask;
  uint8_t axis_missing_mask;
  uint8_t timing_fail_mask;
  uint8_t timing_passed;
} perf_snapshot_t;

enum {
  PERF_TIMING_FAIL_400HZ_BUDGET = (1u << 0),
  PERF_TIMING_FAIL_400HZ_OVERRUN = (1u << 1),
  PERF_TIMING_FAIL_200HZ_OVERRUN = (1u << 2),
  PERF_TIMING_FAIL_1HZ_OVERRUN = (1u << 3),
  PERF_TIMING_FAIL_400HZ_PENDING = (1u << 4),
  PERF_TIMING_FAIL_200HZ_PENDING = (1u << 5),
  PERF_TIMING_FAIL_1HZ_PENDING = (1u << 6),
};

void perf_diag_init(void);
void perf_diag_hook_begin(perf_hook_t hook);
void perf_diag_hook_end(perf_hook_t hook);

void perf_diag_set_scheduler(const scheduler_counters_t *cnt);
void perf_diag_set_can_stats(const can_proto_stats_t *stats);
void perf_diag_set_axis_status(uint8_t axis_id, bool valid, uint32_t age_ms);
void perf_diag_set_imu_repeat(bool sample_valid, bool same_as_prev);

void perf_diag_get_snapshot(perf_snapshot_t *out);
void perf_diag_get_timing_result(uint8_t *out_fail_mask, bool *out_passed);

#endif /* __MASTER_PERF_DIAG_H */
