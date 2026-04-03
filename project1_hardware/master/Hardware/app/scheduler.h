#ifndef __MASTER_SCHEDULER_H
#define __MASTER_SCHEDULER_H
#include <stdbool.h>
#include <stdint.h>

typedef void (*scheduler_hook_t)(void);

void scheduler_init(void);
void scheduler_tick_isr(void); /* call at MASTER_TICK_HZ */
void app_loop(void);           /* execute pending tasks */

void scheduler_register_hooks(scheduler_hook_t hook_400hz, scheduler_hook_t hook_can_200hz);

typedef struct {
  uint32_t cnt_400hz;
  uint32_t cnt_can_200hz;
  uint32_t overrun_400hz;
  uint32_t overrun_can_200hz;
  uint16_t pending_400hz;
  uint16_t pending_can_200hz;
} scheduler_counters_t;

void scheduler_get_counters(scheduler_counters_t *out);

#endif /* __MASTER_SCHEDULER_H */
