#include "mpu6050.h"

#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"

#define MPU6050_I2C_ADDR 0x68u
#define MPU6050_I2C I2C2
#define MPU6050_I2C_CLOCK_HZ 400000u
#define MPU6050_REG_SMPLRT_DIV 0x19u
#define MPU6050_REG_CONFIG 0x1Au
#define MPU6050_REG_GYRO_CFG 0x1Bu
#define MPU6050_REG_ACCEL_CFG 0x1Cu
#define MPU6050_REG_ACCEL_XOUT_H 0x3Bu
#define MPU6050_REG_PWR_MGMT_1 0x6Bu
#define MPU6050_REG_PWR_MGMT_2 0x6Cu

#define MPU6050_SAMPLE_RAW_BYTES 14u
#define MPU6050_I2C_WAIT_LOOPS 10000u

/* STM32F103 DMA1 request mapping: I2C2_RX -> Channel5, I2C2_TX -> Channel4. */
#define MPU6050_DMA_RX_CHANNEL DMA1_Channel5
#define MPU6050_DMA_RX_FLAGS (DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_HT5 | DMA1_FLAG_TE5)
#define MPU6050_DMA_RX_TC_FLAG DMA1_FLAG_TC5
#define MPU6050_DMA_RX_TE_FLAG DMA1_FLAG_TE5

#define MPU6050_ACCEL_LSB_PER_G 2048.0f
#define MPU6050_GYRO_LSB_PER_DPS 16.4f

static imu_sample_t g_bias = {0};
static uint8_t g_raw_dma[MPU6050_SAMPLE_RAW_BYTES] = {0};
static bool g_ready = false;
static bool g_dma_busy = false;

static bool wait_i2c_event(uint32_t event) {
  uint32_t timeout = MPU6050_I2C_WAIT_LOOPS;
  while (I2C_CheckEvent(MPU6050_I2C, event) == ERROR) {
    if (timeout-- == 0u) {
      return false;
    }
  }
  return true;
}

static bool wait_i2c_flag(uint32_t flag, FlagStatus status) {
  uint32_t timeout = MPU6050_I2C_WAIT_LOOPS;
  while (I2C_GetFlagStatus(MPU6050_I2C, flag) != status) {
    if (timeout-- == 0u) {
      return false;
    }
  }
  return true;
}

static void mpu6050_bus_init(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  GPIO_InitTypeDef gpio = {0};
  gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  gpio.GPIO_Mode = GPIO_Mode_AF_OD;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio);

  I2C_DeInit(MPU6050_I2C);
  I2C_InitTypeDef i2c = {0};
  i2c.I2C_ClockSpeed = MPU6050_I2C_CLOCK_HZ;
  i2c.I2C_Mode = I2C_Mode_I2C;
  i2c.I2C_DutyCycle = I2C_DutyCycle_2;
  i2c.I2C_OwnAddress1 = 0x10u;
  i2c.I2C_Ack = I2C_Ack_Enable;
  i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(MPU6050_I2C, &i2c);
  I2C_Cmd(MPU6050_I2C, ENABLE);

  DMA_DeInit(MPU6050_DMA_RX_CHANNEL);
  DMA_InitTypeDef dma = {0};
  dma.DMA_PeripheralBaseAddr = (uint32_t)&MPU6050_I2C->DR;
  dma.DMA_MemoryBaseAddr = (uint32_t)g_raw_dma;
  dma.DMA_DIR = DMA_DIR_PeripheralSRC;
  dma.DMA_BufferSize = MPU6050_SAMPLE_RAW_BYTES;
  dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  dma.DMA_Mode = DMA_Mode_Normal;
  dma.DMA_Priority = DMA_Priority_High;
  dma.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(MPU6050_DMA_RX_CHANNEL, &dma);
  DMA_Cmd(MPU6050_DMA_RX_CHANNEL, DISABLE);
  DMA_ClearFlag(MPU6050_DMA_RX_FLAGS);
}

static bool mpu6050_write_reg(uint8_t reg, uint8_t value) {
  if (!wait_i2c_flag(I2C_FLAG_BUSY, RESET)) {
    return false;
  }

  I2C_GenerateSTART(MPU6050_I2C, ENABLE);
  if (!wait_i2c_event(I2C_EVENT_MASTER_MODE_SELECT)) {
    return false;
  }

  I2C_Send7bitAddress(MPU6050_I2C, (uint8_t)(MPU6050_I2C_ADDR << 1u), I2C_Direction_Transmitter);
  if (!wait_i2c_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    return false;
  }

  I2C_SendData(MPU6050_I2C, reg);
  if (!wait_i2c_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    return false;
  }

  I2C_SendData(MPU6050_I2C, value);
  if (!wait_i2c_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    return false;
  }

  I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
  return true;
}

static void mpu6050_abort_dma_read(void) {
  DMA_Cmd(MPU6050_DMA_RX_CHANNEL, DISABLE);
  DMA_ClearFlag(MPU6050_DMA_RX_FLAGS);
  I2C_DMACmd(MPU6050_I2C, DISABLE);
  I2C_DMALastTransferCmd(MPU6050_I2C, DISABLE);
  I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
  I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
  g_dma_busy = false;
}

static bool mpu6050_begin_dma_read(void) {
  if (!wait_i2c_flag(I2C_FLAG_BUSY, RESET)) {
    return false;
  }

  I2C_GenerateSTART(MPU6050_I2C, ENABLE);
  if (!wait_i2c_event(I2C_EVENT_MASTER_MODE_SELECT)) {
    goto fail;
  }

  I2C_Send7bitAddress(MPU6050_I2C, (uint8_t)(MPU6050_I2C_ADDR << 1u), I2C_Direction_Transmitter);
  if (!wait_i2c_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    goto fail;
  }

  I2C_SendData(MPU6050_I2C, MPU6050_REG_ACCEL_XOUT_H);
  if (!wait_i2c_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    goto fail;
  }

  I2C_GenerateSTART(MPU6050_I2C, ENABLE);
  if (!wait_i2c_event(I2C_EVENT_MASTER_MODE_SELECT)) {
    goto fail;
  }

  I2C_Send7bitAddress(MPU6050_I2C, (uint8_t)(MPU6050_I2C_ADDR << 1u), I2C_Direction_Receiver);
  if (!wait_i2c_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
    goto fail;
  }

  DMA_ClearFlag(MPU6050_DMA_RX_FLAGS);
  DMA_SetCurrDataCounter(MPU6050_DMA_RX_CHANNEL, MPU6050_SAMPLE_RAW_BYTES);
  I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
  I2C_DMALastTransferCmd(MPU6050_I2C, ENABLE);
  I2C_DMACmd(MPU6050_I2C, ENABLE);
  DMA_Cmd(MPU6050_DMA_RX_CHANNEL, ENABLE);
  g_dma_busy = true;
  return true;

fail:
  mpu6050_abort_dma_read();
  return false;
}

static int16_t read_be_i16(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8u) | (uint16_t)p[1]);
}

static void mpu6050_convert_raw_to_sample(const uint8_t raw[MPU6050_SAMPLE_RAW_BYTES],
                                          imu_sample_t *out) {
  if (!raw || !out) {
    return;
  }

  int16_t ax_raw = read_be_i16(&raw[0]);
  int16_t ay_raw = read_be_i16(&raw[2]);
  int16_t az_raw = read_be_i16(&raw[4]);
  int16_t gx_raw = read_be_i16(&raw[8]);
  int16_t gy_raw = read_be_i16(&raw[10]);
  int16_t gz_raw = read_be_i16(&raw[12]);

  out->ax = (float)ax_raw / MPU6050_ACCEL_LSB_PER_G;
  out->ay = (float)ay_raw / MPU6050_ACCEL_LSB_PER_G;
  out->az = (float)az_raw / MPU6050_ACCEL_LSB_PER_G;
  out->gx = (float)gx_raw / MPU6050_GYRO_LSB_PER_DPS;
  out->gy = (float)gy_raw / MPU6050_GYRO_LSB_PER_DPS;
  out->gz = (float)gz_raw / MPU6050_GYRO_LSB_PER_DPS;
  out->gx -= g_bias.gx;
  out->gy -= g_bias.gy;
  out->gz -= g_bias.gz;
}

bool mpu6050_init(void) {
  mpu6050_bus_init();

  if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01u)) {
    g_ready = false;
    return false;
  }
  if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_2, 0x00u)) {
    g_ready = false;
    return false;
  }
  if (!mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 0x00u)) {
    g_ready = false;
    return false;
  }
  if (!mpu6050_write_reg(MPU6050_REG_CONFIG, 0x02u)) {
    g_ready = false;
    return false;
  }
  if (!mpu6050_write_reg(MPU6050_REG_GYRO_CFG, 0x18u)) {
    g_ready = false;
    return false;
  }
  if (!mpu6050_write_reg(MPU6050_REG_ACCEL_CFG, 0x18u)) {
    g_ready = false;
    return false;
  }

  g_ready = true;
  g_dma_busy = false;
  g_bias = (imu_sample_t){0};
  return true;
}

bool mpu6050_is_ready(void) {
  return g_ready;
}

bool mpu6050_start_read_dma(void) {
  if (!g_ready) {
    return false;
  }
  if (g_dma_busy) {
    return true;
  }
  return mpu6050_begin_dma_read();
}

bool mpu6050_poll_sample_dma(imu_sample_t *out) {
  if (!g_ready || !out || !g_dma_busy) {
    return false;
  }

  if (DMA_GetFlagStatus(MPU6050_DMA_RX_TE_FLAG) != RESET) {
    mpu6050_abort_dma_read();
    return false;
  }
  if (DMA_GetFlagStatus(MPU6050_DMA_RX_TC_FLAG) == RESET) {
    return false;
  }

  DMA_Cmd(MPU6050_DMA_RX_CHANNEL, DISABLE);
  DMA_ClearFlag(MPU6050_DMA_RX_FLAGS);
  I2C_DMACmd(MPU6050_I2C, DISABLE);
  I2C_DMALastTransferCmd(MPU6050_I2C, DISABLE);
  I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
  I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
  g_dma_busy = false;

  mpu6050_convert_raw_to_sample(g_raw_dma, out);
  return true;
}

bool mpu6050_read_sample(imu_sample_t *out) {
  return mpu6050_poll_sample_dma(out);
}

void mpu6050_set_bias(const imu_sample_t *bias) {
  if (!g_ready || !bias) {
    return;
  }
  g_bias.gx = bias->gx;
  g_bias.gy = bias->gy;
  g_bias.gz = bias->gz;
}

