#ifndef __MAHONY_H
#define __MAHONY_H
#include "types.h"

void mahony_init(float kp, float ki);
void mahony_update(const imu_sample_t *s, float dt, quat_t *q_inout);
void quat_to_euler(const quat_t *q, float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif /* __MAHONY_H */

