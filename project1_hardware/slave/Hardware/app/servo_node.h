#ifndef SERVO_NODE_H
#define SERVO_NODE_H

#include <stdint.h>

#include "../common/include/types.h"

typedef struct {
  uint16_t seq_echo;
  uint16_t pulse_us;
  int16_t cur_pos_cdeg;
  uint8_t fault;
  uint8_t vbat_01v;
  slave_state_t state;
} servo_node_status_t;

void servo_node_init(void);
void servo_node_step_200hz(float dt_s);
void servo_node_on_timeout(void);

void servo_node_set_time_ms(uint32_t now_ms);
void servo_node_update_vbat(float vbat_v);
void servo_node_get_status(servo_node_status_t *out);

#endif /* SERVO_NODE_H */

