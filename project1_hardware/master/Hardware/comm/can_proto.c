#include "can_proto.h"

#include <string.h>

#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

static can_status_t g_status_cache[4];
static bool g_status_valid[4] = {false, false, false, false};
static uint32_t g_status_last_ms[4] = {0u, 0u, 0u, 0u};
static uint32_t g_now_ms = 0u;
static can_proto_stats_t g_stats = {0};

static uint8_t status_axis_from_id(uint32_t std_id) {
  if (std_id < CAN_ID_STATUS_BASE) {
    return 0u;
  }
  const uint32_t axis_id = std_id - CAN_ID_STATUS_BASE;
  if (axis_id < 1u || axis_id > 3u) {
    return 0u;
  }
  return (uint8_t)axis_id;
}

static void can_proto_hw_init(void) {
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
  can.CAN_Prescaler = 48u; /* example timing: 36MHz / 48 / (1 + 2 + 3) = 125kbps */
  (void)CAN_Init(CAN1, &can);

  CAN_FilterInitTypeDef filter = {0};
  filter.CAN_FilterNumber = 0u;
  filter.CAN_FilterMode = CAN_FilterMode_IdMask;
  filter.CAN_FilterScale = CAN_FilterScale_32bit;
  filter.CAN_FilterIdHigh = (uint16_t)(CAN_ID_STATUS_BASE << 5u);
  filter.CAN_FilterIdLow = 0x0000u;
  filter.CAN_FilterMaskIdHigh = (uint16_t)(0x700u << 5u); /* accept 0x2xx status frames */
  filter.CAN_FilterMaskIdLow = 0x0000u;
  filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
  filter.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&filter);
}

void can_proto_init(void) {
  can_proto_hw_init();
  memset(g_status_cache, 0, sizeof(g_status_cache));
  g_status_valid[0] = false;
  g_status_valid[1] = false;
  g_status_valid[2] = false;
  g_status_valid[3] = false;
  g_status_last_ms[0] = 0u;
  g_status_last_ms[1] = 0u;
  g_status_last_ms[2] = 0u;
  g_status_last_ms[3] = 0u;
  g_now_ms = 0u;
  g_stats = (can_proto_stats_t){0};
}

bool can_send_cmd(uint16_t seq, int16_t yaw_cdeg, int16_t pitch_cdeg, int16_t roll_cdeg) {
  uint8_t frame_data[8] = {0};
  const can_cmd_t cmd = {
      .seq = seq,
      .yaw_ref_cdeg = yaw_cdeg,
      .pitch_ref_cdeg = pitch_cdeg,
      .roll_ref_cdeg = roll_cdeg,
  };
  if (!can_pack_cmd(&cmd, frame_data)) {
    ++g_stats.cmd_pack_fail;
    return false;
  }
  CanTxMsg tx = {0};
  tx.StdId = CAN_ID_CMD;
  tx.ExtId = 0u;
  tx.IDE = CAN_Id_Standard;
  tx.RTR = CAN_RTR_Data;
  tx.DLC = 8u;
  for (uint8_t i = 0; i < 8u; ++i) {
    tx.Data[i] = frame_data[i];
  }

  const uint8_t mailbox = CAN_Transmit(CAN1, &tx);
  if (mailbox == CAN_TxStatus_NoMailBox) {
    ++g_stats.cmd_tx_drop_no_mailbox;
    return false;
  }
  ++g_stats.cmd_tx_ok;
  return true;
}

void can_proto_set_time_ms(uint32_t now_ms) { g_now_ms = now_ms; }

void can_proto_poll_status(void) {
  while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0u) {
    CanRxMsg rx = {0};
    CAN_Receive(CAN1, CAN_FIFO0, &rx);

    if (rx.IDE != CAN_Id_Standard || rx.DLC != 8u) {
      ++g_stats.status_rx_bad_id_or_dlc;
      continue;
    }

    const uint8_t axis_id = status_axis_from_id(rx.StdId);
    if (axis_id == 0u) {
      ++g_stats.status_rx_bad_id_or_dlc;
      continue;
    }

    can_status_t st = {0};
    if (!can_unpack_status(rx.Data, rx.DLC, &st)) {
      ++g_stats.status_rx_unpack_fail;
      continue;
    }

    g_status_cache[axis_id] = st;
    g_status_valid[axis_id] = true;
    g_status_last_ms[axis_id] = g_now_ms;
    ++g_stats.status_rx_ok;
  }
}

bool can_proto_get_latest_status(uint8_t axis_id, can_status_t *out_status, uint32_t *out_age_ms) {
  if (axis_id < 1u || axis_id > 3u || !out_status || !g_status_valid[axis_id]) {
    return false;
  }
  *out_status = g_status_cache[axis_id];
  if (out_age_ms) {
    *out_age_ms = g_now_ms - g_status_last_ms[axis_id];
  }
  return true;
}

void can_proto_get_stats(can_proto_stats_t *out_stats) {
  if (!out_stats) {
    return;
  }
  *out_stats = g_stats;
}
