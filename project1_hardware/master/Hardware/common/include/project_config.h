#ifndef __MASTER_PROJECT_CONFIG_H
#define __MASTER_PROJECT_CONFIG_H

/* 2.5 ms base tick driven by TIM2 update interrupt. */
#define MASTER_TICK_HZ 400u
#define MASTER_TICK_PERIOD_US (1000000u / MASTER_TICK_HZ)

#define MASTER_TASK_400HZ_DIV 1u
#define MASTER_TASK_CAN_200HZ_DIV 2u

#define MASTER_CTRL_DT_S ((float)MASTER_TICK_PERIOD_US * 1.0e-6f)
#define MASTER_CTRL_DT_MIN_S 0.001f
#define MASTER_CTRL_DT_MAX_S 0.020f

#if ((1000000u % MASTER_TICK_HZ) != 0u)
#error "MASTER_TICK_HZ must divide 1,000,000 exactly for integer microsecond tick period."
#endif

#if (MASTER_TASK_400HZ_DIV == 0u) || (MASTER_TASK_CAN_200HZ_DIV == 0u)
#error "Task divisors must be non-zero."
#endif

#endif /* __MASTER_PROJECT_CONFIG_H */
