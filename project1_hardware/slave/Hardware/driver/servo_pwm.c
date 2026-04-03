#include "servo_pwm.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#define SERVO_PWM_MIN_US 500u
#define SERVO_PWM_MAX_US 2500u

static uint16_t g_pwm_hz = 50u;
static uint16_t g_pulse_us = 1500u;
static uint16_t g_period_ticks = 20000u;

static void tim_set_compare(uint16_t ticks) {
  const uint16_t arr_max = (g_period_ticks > 0u) ? (uint16_t)(g_period_ticks - 1u) : 0u;
  if (ticks > arr_max) {
    ticks = arr_max;
  }
  TIM_SetCompare1(TIM3, ticks);
}

void servo_pwm_init(uint16_t pwm_hz) {
  if (pwm_hz == 0u) {
    pwm_hz = 50u;
  }
  g_pwm_hz = pwm_hz;
  g_pulse_us = 1500u;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  GPIO_InitTypeDef gpio = {0};
  gpio.GPIO_Pin = GPIO_Pin_6;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio);

  TIM_DeInit(TIM3);

  RCC_ClocksTypeDef clocks = {0};
  RCC_GetClocksFreq(&clocks);
  uint32_t tim_clk_hz = clocks.PCLK1_Frequency;
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
    tim_clk_hz *= 2u;
  }
  uint32_t psc_u32 = tim_clk_hz / 1000000u;
  if (psc_u32 == 0u) {
    psc_u32 = 1u;
  }
  const uint16_t psc = (uint16_t)(psc_u32 - 1u);
  uint32_t period_ticks_u32 = 1000000u / g_pwm_hz;
  if (period_ticks_u32 < 1000u) {
    period_ticks_u32 = 1000u;
  } else if (period_ticks_u32 > 65535u) {
    period_ticks_u32 = 65535u;
  }
  g_period_ticks = (uint16_t)period_ticks_u32;

  TIM_TimeBaseInitTypeDef tb = {0};
  tb.TIM_Period = (uint16_t)(g_period_ticks - 1u);
  tb.TIM_Prescaler = psc;
  tb.TIM_ClockDivision = 0u;
  tb.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &tb);

  TIM_OCInitTypeDef oc = {0};
  TIM_OCStructInit(&oc);
  oc.TIM_OCMode = TIM_OCMode_PWM1;
  oc.TIM_OutputState = TIM_OutputState_Enable;
  oc.TIM_OCPolarity = TIM_OCPolarity_High;
  oc.TIM_Pulse = g_pulse_us;

  TIM_OC1Init(TIM3, &oc);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  tim_set_compare(g_pulse_us);
}

void servo_pwm_set_pulse_us(uint16_t us) {
  if (us < SERVO_PWM_MIN_US) {
    us = SERVO_PWM_MIN_US;
  }
  if (us > SERVO_PWM_MAX_US) {
    us = SERVO_PWM_MAX_US;
  }
  g_pulse_us = us;
  tim_set_compare(g_pulse_us);
}

uint16_t servo_pwm_get_pulse_us(void) { return g_pulse_us; }

uint16_t servo_pwm_get_pwm_hz(void) { return g_pwm_hz; }
