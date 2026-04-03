#ifndef __MPU6050_H
#define __MPU6050_H
#include <stdbool.h>
#include <stdint.h>

#include "types.h"

bool mpu6050_init(void);
bool mpu6050_is_ready(void);
bool mpu6050_start_read_dma(void);
bool mpu6050_poll_sample_dma(imu_sample_t *out);
bool mpu6050_read_sample(imu_sample_t *out);
void mpu6050_set_bias(const imu_sample_t *bias);

#endif /* __MPU6050_H */
