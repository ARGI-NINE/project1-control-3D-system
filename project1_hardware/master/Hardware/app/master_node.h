#ifndef __MASTER_NODE_H
#define __MASTER_NODE_H
#include <stdbool.h>

#include "types.h"

void master_node_init(void);
void master_node_select_controller(bool use_algo_b);
void master_node_set_target_deg(float yaw_deg, float pitch_deg, float roll_deg);
void master_node_step_400hz(const imu_sample_t *imu, bool imu_valid, float dt);
void master_node_step_can_200hz(void);

master_state_t master_node_get_state(void);
void master_node_get_pos_ref_deg(float *yaw_deg, float *pitch_deg, float *roll_deg);

#endif /* __MASTER_NODE_H */
