#include "can_rx.h"

#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "../common/include/can_protocol.h"
#include "../config/axis_config.h"

static int16_t g_latest_target_cdeg = 0;
static uint16_t g_latest_seq = 0;
static bool g_has_target = false;
static uint32_t g_now_ms = 0;
static uint32_t g_last_rx_ms = 0;
static can_rx_stats_t g_stats = {0};
static bool g_seq_inited = false;

static void can_rx_hw_init(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1, DISABLE); /* use PA11/PA12 */

  GPIO_InitTypeDef gpio = {0};
  gpio.GPIO_Pin = GPIO_Pin_11;
  gpio.GPIO_Mode = GPIO_Mode_IPU;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio);

  gpio.GPIO_Pin = GPIO_Pin_12;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio);

  CAN_DeInit(CAN1);
  CAN_InitTypeDef can = {0};
  CAN_StructInit(&can);
  can.CAN_TTCM = DISABLE;
  can.CAN_ABOM = DISABLE;
  can.CAN_AWUM = DISABLE;
  can.CAN_NART = DISABLE;
  can.CAN_RFLM = DISABLE;
  can.CAN_TXFP = DISABLE;
  can.CAN_Mode = CAN_Mode_Normal;
  can.CAN_SJW = CAN_SJW_2tq;
  can.CAN_BS1 = CAN_BS1_2tq;
  can.CAN_BS2 = CAN_BS2_3tq;
  can.CAN_Prescaler = 48u; /* 36MHz / 48 / (1 + 2 + 3) = 125kbps */
  (void)CAN_Init(CAN1, &can);

  CAN_FilterInitTypeDef filter = {0};
  filter.CAN_FilterNumber = 0u;
  filter.CAN_FilterMode = CAN_FilterMode_IdMask;
  filter.CAN_FilterScale = CAN_FilterScale_32bit;
  filter.CAN_FilterIdHigh = (uint16_t)(CAN_ID_CMD << 5u);
  filter.CAN_FilterIdLow = 0x0000u; /* IDE=0, RTR=0 */
  filter.CAN_FilterMaskIdHigh = (uint16_t)(0x7FFu << 5u);
  filter.CAN_FilterMaskIdLow = 0x0006u; /* match IDE/RTR bits */
  filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
  filter.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&filter);
}

void can_rx_init(void) {
  can_rx_hw_init();
  g_latest_target_cdeg = 0;
  g_latest_seq = 0;
  g_has_target = false;
  g_now_ms = 0;
  g_last_rx_ms = 0;
  g_stats = (can_rx_stats_t){0};
  g_seq_inited = false;
}

void can_rx_set_time_ms(uint32_t now_ms) { g_now_ms = now_ms; }

uint32_t can_rx_ms_since_last_msg(void) { return g_now_ms - g_last_rx_ms; }

void can_rx_poll_hw(void) {
  CanRxMsg latest_cmd = {0};
  uint32_t valid_cmd_count = 0u;
  while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0u) {
    CanRxMsg msg = {0};
    CAN_Receive(CAN1, CAN_FIFO0, &msg);
    ++g_stats.rx_total;
    if (msg.IDE != CAN_Id_Standard || msg.RTR != CAN_RTR_Data || msg.StdId != CAN_ID_CMD ||
        msg.DLC != 8u) {
      ++g_stats.rx_bad_id_or_dlc;
      continue;
    }
    latest_cmd = msg;
    ++valid_cmd_count;
  }

  if (valid_cmd_count > 1u) {
    g_stats.rx_cmd_dropped_backlog += (valid_cmd_count - 1u);
  }
  if (valid_cmd_count > 0u) {
    can_rx_isr_on_msg(latest_cmd.StdId, latest_cmd.RTR, latest_cmd.Data, latest_cmd.DLC);
  }
}

void can_rx_isr_on_msg(uint32_t id, uint8_t rtr, const uint8_t *data, uint8_t dlc) {
  if (id != CAN_ID_CMD || rtr != CAN_RTR_Data || !data || dlc != 8u) {
    ++g_stats.rx_bad_id_or_dlc;
    return;
  }

  can_cmd_t cmd;
  if (!can_unpack_cmd(data, dlc, &cmd)) {
    ++g_stats.rx_unpack_fail;
    return;
  }

  const slave_axis_config_t *cfg = slave_axis_config_get();
  int16_t axis_cdeg = 0;
  if (cfg->axis == AXIS_YAW) {
    axis_cdeg = cmd.yaw_ref_cdeg;
  } else if (cfg->axis == AXIS_PITCH) {
    axis_cdeg = cmd.pitch_ref_cdeg;
  } else {
    axis_cdeg = cmd.roll_ref_cdeg;
  }

  g_latest_target_cdeg = axis_cdeg;
  if (g_seq_inited && (uint16_t)(g_latest_seq + 1u) != cmd.seq) {
    ++g_stats.rx_seq_jump;
  }
  g_latest_seq = cmd.seq;
  g_seq_inited = true;
  g_has_target = true;
  g_last_rx_ms = g_now_ms;
  ++g_stats.rx_accepted;
}

bool can_get_latest_target(int16_t *out_cdeg, uint16_t *out_seq) {
  if (!g_has_target) {
    return false;
  }
  if (out_cdeg) {
    *out_cdeg = g_latest_target_cdeg;
  }
  if (out_seq) {
    *out_seq = g_latest_seq;
  }
  return true;
}

bool can_tx_status(uint8_t axis_id, const can_status_t *st) {
  if (!st) {
    return false;
  }
  if (axis_id < 1u || axis_id > 3u) {
    axis_id = 1u;
  }

  uint8_t frame_data[8] = {0};
  if (!can_pack_status(st, frame_data)) {
    return false;
  }

  CanTxMsg tx = {0};
  tx.StdId = (uint16_t)(CAN_ID_STATUS_BASE + axis_id);
  tx.ExtId = 0u;
  tx.IDE = CAN_Id_Standard;
  tx.RTR = CAN_RTR_Data;
  tx.DLC = 8u;
  for (uint8_t i = 0; i < 8u; ++i) {
    tx.Data[i] = frame_data[i];
  }

  const uint8_t mailbox = CAN_Transmit(CAN1, &tx);
  if (mailbox == CAN_TxStatus_NoMailBox) {
    ++g_stats.status_tx_drop_no_mailbox;
    return false;
  }
  ++g_stats.status_tx_ok;
  return true;
}

void can_rx_get_stats(can_rx_stats_t *out_stats) {
  if (!out_stats) {
    return;
  }
  *out_stats = g_stats;
}
