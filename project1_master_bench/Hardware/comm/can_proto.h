#ifndef __MASTER_CAN_PROTO_H
#define __MASTER_CAN_PROTO_H
#include <stdbool.h>
#include <stdint.h>

#include "types.h"
#include "can_protocol.h"

void can_proto_init(void);
bool can_send_cmd(uint16_t seq, int16_t yaw_cdeg, int16_t pitch_cdeg, int16_t roll_cdeg);

void can_proto_set_time_ms(uint32_t now_ms);
void can_proto_poll_status(void);
bool can_proto_get_latest_status(uint8_t axis_id, can_status_t *out_status, uint32_t *out_age_ms);

typedef struct {
  uint32_t cmd_tx_ok;
  uint32_t cmd_tx_drop_no_mailbox;
  uint32_t cmd_pack_fail;
  uint32_t status_rx_ok;
  uint32_t status_rx_bad_id_or_dlc;
  uint32_t status_rx_unpack_fail;
} can_proto_stats_t;

void can_proto_get_stats(can_proto_stats_t *out_stats);

#endif /* __MASTER_CAN_PROTO_H */
