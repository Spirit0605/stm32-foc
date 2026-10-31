#include "AS5600.h"
#include "main.h"
#include <math.h>

AS5600_DataTypeDef encoder;
static volatile uint16_t prev_raw = 0;          //上一次的原始角度值
static uint32_t last_tick = 0;                  //上一次采样的时间戳
static uint8_t has_reference = 0;               //是否已经建立参考基准

/**
 * @brief  读取 AS5600 原始角度寄存器
 * @return 1:成功  0:失败
 */
static uint8_t read_raw_angle(uint16_t *raw)
{
  uint8_t buf[2];

  if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS) != HAL_OK)
    return 0;

  *raw = ((uint16_t)buf[0] << 8 | buf[1]) & 0x0FFF;
  return 1;
}

/**
 * @brief  初始化编码器
 */
uint8_t as5600_init(void)
{
  uint16_t raw;

  encoder.raw = 0;
  encoder.total = 0;
  encoder.rpm = 0.0f;
  encoder.mech_angle = 0.0f;
  encoder.elec_angle = 0.0f;
  has_reference = 0;

  if (read_raw_angle(&raw) == 0)
    return 0;

  encoder.raw = raw;
  prev_raw = raw;
  last_tick = HAL_GetTick();
  has_reference = 1;

  return 1;
}

/**
 * @brief  更新编码器数据（角度 + 转速）
 * @note   建议以固定周期调用（例如 10ms）
 * @return 1:成功  0:I2C通信失败
 */
uint8_t as5600_update(void)
{
  uint16_t raw;
  int32_t delta;
  uint32_t now, dt;
  float rpm_instant;

  // 1. 读取原始角度
  if (read_raw_angle(&raw) == 0)
  {
    has_reference = 0; // 通信失败后，下次重新建立基准
    encoder.rpm = 0.0f;
    return 0;
  }

  encoder.raw = raw;

  // 2. 第一次有效数据 or 通信恢复后，只建立基准，不计算速度
  if (has_reference == 0)
  {
    prev_raw = raw;
    last_tick = HAL_GetTick();
    has_reference = 1;
    return 1;
  }

  // 3. 计算角度增量，并处理过零回绕（Wrap-around）
  delta = (int32_t)raw - (int32_t)prev_raw;

  if (delta > (AS5600_RESOLUTION / 2)) // 正向过零
    delta -= AS5600_RESOLUTION;
  else if (delta < -(AS5600_RESOLUTION / 2)) // 反向过零
    delta += AS5600_RESOLUTION;

  prev_raw = raw;
  encoder.total += delta; // 多圈累计

  // 4. 计算瞬时转速并进行一阶低通滤波
  now = HAL_GetTick();
  dt = now - last_tick;
  last_tick = now;

  if (dt > 0)
  {
    rpm_instant = (float)delta * 60000.0f /
                  ((float)AS5600_RESOLUTION * (float)dt);

    // 一阶低通：y = y + α * (x - y)
    encoder.rpm += SPEED_LPF_ALPHA * (rpm_instant - encoder.rpm);
  }

  // 5. 计算机械角度与电角度（单位：弧度）
  encoder.mech_angle = (float)raw * TWO_PI / AS5600_RESOLUTION;

  encoder.elec_angle = encoder.mech_angle * MOTOR_POLE_PAIRS;

  // 归一化到 [0, 2π)
  encoder.elec_angle = fmodf(encoder.elec_angle, TWO_PI);
  if (encoder.elec_angle < 0.0f)
    encoder.elec_angle += TWO_PI;

  return 1;
}

/**
 * @brief  获取当前编码器数据
 */
void as5600_get_data(AS5600_DataTypeDef *data)
{
  if (data != NULL)
    *data = encoder;
}