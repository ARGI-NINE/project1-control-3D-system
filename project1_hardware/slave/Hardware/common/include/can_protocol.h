#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "types.h"

enum {
  CAN_ID_CMD = 0x100,
  CAN_ID_STATUS_BASE = 0x200
};

typedef struct {
  uint16_t seq_echo;
  uint16_t pulse_us;
  int16_t cur_pos_cdeg;
  uint8_t fault;
  uint8_t vbat_01v;
} can_status_t;

bool can_pack_cmd(const can_cmd_t *cmd, uint8_t out_data[8]);
bool can_unpack_cmd(const uint8_t data[8], uint8_t dlc, can_cmd_t *cmd_out);

bool can_pack_status(const can_status_t *st, uint8_t out_data[8]);
bool can_unpack_status(const uint8_t data[8], uint8_t dlc, can_status_t *st_out);

#endif /* CAN_PROTOCOL_H */

