#include "scheduler.h"
#include "project_config.h"
#include "stm32f10x.h"

static volatile uint16_t g_pending_400hz = 0u;
static volatile uint16_t g_pending_200hz = 0u;
static volatile uint16_t g_pending_1hz = 0u;
static volatile uint8_t g_drop_tick_while_log = 0u;
static uint16_t g_countdown_400hz = MASTER_TASK_400HZ_DIV;
static uint16_t g_countdown_200hz = MASTER_TASK_CAN_200HZ_DIV;
static uint16_t g_countdown_1hz = MASTER_TASK_LOG_1HZ_DIV;

static scheduler_hook_t g_hook_400hz = 0;
static scheduler_hook_t g_hook_200hz = 0;
static scheduler_hook_t g_hook_1hz = 0;
static scheduler_counters_t g_cnt = {0};

static void scheduler_inc_pending(volatile uint16_t *pending, uint32_t *overrun_counter) {
  if (!pending || !overrun_counter) {
    return;
  }
  if (*pending < UINT16_MAX) {
    if (*pending > 0u) {
      ++(*overrun_counter);
    }
    ++(*pending);
  } else {
    ++(*overrun_counter);
  }
}

static bool scheduler_take_pending(volatile uint16_t *pending) {
  bool ready = false;
  __disable_irq();
  if (*pending > 0u) {
    *pending = 0u;
    ready = true;
  }
  __enable_irq();
  return ready;
}

void scheduler_init(void) {
  g_pending_400hz = 0u;
  g_pending_200hz = 0u;
  g_pending_1hz = 0u;
  g_drop_tick_while_log = 0u;
  g_countdown_400hz = MASTER_TASK_400HZ_DIV;
  g_countdown_200hz = MASTER_TASK_CAN_200HZ_DIV;
  g_countdown_1hz = MASTER_TASK_LOG_1HZ_DIV;
  g_cnt = (scheduler_counters_t){0};
}

void scheduler_register_hooks(scheduler_hook_t hook_400hz, scheduler_hook_t hook_can_200hz,
                              scheduler_hook_t hook_log_1hz) {
  g_hook_400hz = hook_400hz;
  g_hook_200hz = hook_can_200hz;
  g_hook_1hz = hook_log_1hz;
}

void scheduler_tick_isr(void) {
  if (g_drop_tick_while_log != 0u) {
    return;
  }

  if (g_countdown_400hz > 1u) {
    --g_countdown_400hz;
  } else {
    g_countdown_400hz = MASTER_TASK_400HZ_DIV;
    scheduler_inc_pending(&g_pending_400hz, &g_cnt.overrun_400hz);
  }

  if (g_countdown_200hz > 1u) {
    --g_countdown_200hz;
  } else {
    g_countdown_200hz = MASTER_TASK_CAN_200HZ_DIV;
    scheduler_inc_pending(&g_pending_200hz, &g_cnt.overrun_can_200hz);
  }

  if (g_countdown_1hz > 1u) {
    --g_countdown_1hz;
  } else {
    g_countdown_1hz = MASTER_TASK_LOG_1HZ_DIV;
    scheduler_inc_pending(&g_pending_1hz, &g_cnt.overrun_log_1hz);
  }
}

void app_loop(void) {
  if (scheduler_take_pending(&g_pending_400hz)) {
    ++g_cnt.cnt_400hz;
    if (g_hook_400hz) {
      g_hook_400hz();
    }
  }

  if (scheduler_take_pending(&g_pending_200hz)) {
    ++g_cnt.cnt_can_200hz;
    if (g_hook_200hz) {
      g_hook_200hz();
    }
  }

  if (scheduler_take_pending(&g_pending_1hz)) {
    ++g_cnt.cnt_log_1hz;
    if (g_hook_1hz) {
      __disable_irq();
      g_drop_tick_while_log = 1u;
      __enable_irq();

      g_hook_1hz();

      __disable_irq();
      g_drop_tick_while_log = 0u;
      __enable_irq();
    }
  }
}

void scheduler_get_counters(scheduler_counters_t *out) {
  if (!out) {
    return;
  }
  __disable_irq();
  *out = g_cnt;
  out->pending_400hz = g_pending_400hz;
  out->pending_can_200hz = g_pending_200hz;
  out->pending_log_1hz = g_pending_1hz;
  __enable_irq();
}
