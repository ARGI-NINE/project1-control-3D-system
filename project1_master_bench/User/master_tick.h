#ifndef __MASTER_TICK_H
#define __MASTER_TICK_H
#ifdef __cplusplus
extern "C" {
#endif

/* Call from the timer ISR once per MASTER_TICK_HZ update. */
void master_tick_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* __MASTER_TICK_H */
