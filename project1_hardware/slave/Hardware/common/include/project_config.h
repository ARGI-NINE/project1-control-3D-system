#ifndef SLAVE_PROJECT_CONFIG_H
#define SLAVE_PROJECT_CONFIG_H

/* 5ms base tick driven by TIM2 update interrupt. */
#define SLAVE_TICK_HZ 200u
#define SLAVE_TICK_PERIOD_US (1000000u / SLAVE_TICK_HZ)
#define SLAVE_MS_PER_TICK (SLAVE_TICK_PERIOD_US / 1000u)

#define SLAVE_AXIS_INDEX 1 /* 0:yaw, 1:pitch, 2:roll */
#define SLAVE_SERVO_PWM_HZ 50u
#define SLAVE_CTRL_DT_S 0.005f
#define SLAVE_CTRL_DT_MIN_S 0.001f
#define SLAVE_CTRL_DT_MAX_S 0.050f
#define SLAVE_TIMEOUT_MS 100u
#define SLAVE_TIMEOUT_RETURN_S 0.2f
#define SLAVE_RETURN_CENTER_ON_TIMEOUT 1u
#define SLAVE_VBAT_LOW_V 4.7f

#if ((1000000u % SLAVE_TICK_HZ) != 0u)
#error "SLAVE_TICK_HZ must divide 1,000,000 exactly for integer microsecond tick period."
#endif

#if ((SLAVE_TICK_PERIOD_US % 1000u) != 0u)
#error "SLAVE_TICK_PERIOD_US must be a whole number of milliseconds."
#endif

#if (SLAVE_MS_PER_TICK == 0u)
#error "SLAVE_MS_PER_TICK must be non-zero."
#endif

#endif /* SLAVE_PROJECT_CONFIG_H */
