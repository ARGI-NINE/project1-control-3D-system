#ifndef TYPES_H
#define TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} imu_sample_t; /* acc: g, gyro: deg/s */

typedef struct {
  float w;
  float x;
  float y;
  float z;
} quat_t;

typedef enum {
  AXIS_YAW = 0,
  AXIS_PITCH = 1,
  AXIS_ROLL = 2
} axis_t;

typedef struct {
  int16_t yaw_ref_cdeg;
  int16_t pitch_ref_cdeg;
  int16_t roll_ref_cdeg;
  uint16_t seq;
} can_cmd_t;

typedef enum {
  MASTER_INIT = 0,
  MASTER_CALIB = 1,
  MASTER_RUN = 2,
  MASTER_SAFE = 3
} master_state_t;

typedef enum {
  SLAVE_INIT = 0,
  SLAVE_RUN = 1,
  SLAVE_TIMEOUT = 2,
  SLAVE_FAULT = 3
} slave_state_t;

typedef enum {
  SLAVE_FAULT_OK = 0,
  SLAVE_FAULT_TIMEOUT = 1,
  SLAVE_FAULT_LIMIT = 2,
  SLAVE_FAULT_VBAT_LOW = 3
} slave_fault_t;

#endif /* TYPES_H */

