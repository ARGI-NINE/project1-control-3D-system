#include <stdbool.h>
#include <stdint.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#include "../Hardware/app/servo_node.h"
#include "../Hardware/comm/can_rx.h"
#include "../Hardware/common/include/project_config.h"

static volatile uint32_t g_ms_ticks = 0u;
static volatile uint16_t g_pending_200hz = 0u;
static volatile uint32_t g_overrun_200hz = 0u;

static bool take_pending_200hz(void) {
  bool ready = false;
  __disable_irq();
  if (g_pending_200hz > 0u) {
    g_pending_200hz = 0u;
    ready = true;
  }
  __enable_irq();
  return ready;
}

__weak float board_read_vbat_v(void) {
  /* Replace with ADC sampling in your board layer. */
  return 6.0f;
}

static void slave_tick_isr(void) {
  g_ms_ticks += SLAVE_MS_PER_TICK;
  if (g_pending_200hz > 0u) {
    ++g_overrun_200hz;
  }
  if (g_pending_200hz < UINT16_MAX) {
    ++g_pending_200hz;
  } else {
    ++g_overrun_200hz;
  }
}

static void slave_tim2_init(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_InternalClockConfig(TIM2);

  TIM_TimeBaseInitTypeDef tim_base = {0};
  tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
  tim_base.TIM_CounterMode = TIM_CounterMode_Up;
  tim_base.TIM_Prescaler = (uint16_t)(72u - 1u); /* 72MHz timer clock -> 1MHz */
  tim_base.TIM_Period = (uint16_t)(SLAVE_TICK_PERIOD_US - 1u);
  tim_base.TIM_RepetitionCounter = 0u;
  TIM_TimeBaseInit(TIM2, &tim_base);

  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitTypeDef nvic = {0};
  nvic.NVIC_IRQChannel = TIM2_IRQn;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  nvic.NVIC_IRQChannelPreemptionPriority = 1u;
  nvic.NVIC_IRQChannelSubPriority = 0u;
  NVIC_Init(&nvic);

  TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    slave_tick_isr();
  }
}

int main(void) {
  servo_node_init();
  slave_tim2_init();

  while (1) {
    can_rx_poll_hw();
    if (take_pending_200hz()) {
      servo_node_set_time_ms(g_ms_ticks);
      servo_node_update_vbat(board_read_vbat_v());
      servo_node_step_200hz(SLAVE_CTRL_DT_S);
    }
  }
}
