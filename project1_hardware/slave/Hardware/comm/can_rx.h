#ifndef SLAVE_CAN_RX_H
#define SLAVE_CAN_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/include/can_protocol.h"

typedef struct {
  uint32_t rx_total;
  uint32_t rx_accepted;
  uint32_t rx_cmd_dropped_backlog;
  uint32_t rx_bad_id_or_dlc;
  uint32_t rx_unpack_fail;
  uint32_t rx_seq_jump;
  uint32_t status_tx_ok;
  uint32_t status_tx_drop_no_mailbox;
} can_rx_stats_t;

void can_rx_init(void);
void can_rx_poll_hw(void);
void can_rx_isr_on_msg(uint32_t id, uint8_t rtr, const uint8_t *data, uint8_t dlc);
bool can_get_latest_target(int16_t *out_cdeg, uint16_t *out_seq);
bool can_tx_status(uint8_t axis_id, const can_status_t *st);
void can_rx_get_stats(can_rx_stats_t *out_stats);

void can_rx_set_time_ms(uint32_t now_ms);
uint32_t can_rx_ms_since_last_msg(void);

#endif /* SLAVE_CAN_RX_H */
