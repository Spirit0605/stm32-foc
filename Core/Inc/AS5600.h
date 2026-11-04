#ifndef __AS5600_H
#define __AS5600_H

#include <stdint.h>
#include "i2c.h"

/* AS5600 I2C 地址与寄存器 */
#define AS5600_ADDR (0x36U << 1)
#define AS5600_REG_RAW_ANGLE 0x0CU

#define AS5600_RESOLUTION 4096.0f // 12bit 分辨率
#define AS5600_SAMPLE_PERIOD_MS 1 // 采样周期

#define MOTOR_POLE_PAIRS 7.0f // 电机极对数

#define I2C_TIMEOUT_MS 8
#define SPEED_LPF_ALPHA 0.10f // 转速一阶低通系数 (0~1，越大越跟随，越小越平滑)
#define PI 3.1415926535f
#define TWO_PI (2.0f * PI) // 2*3.14

typedef struct
{
    uint16_t raw;     // 原始角度值 (0 ~ 4095)
    int32_t total;    // 多圈累计角度（编码器计数）
    float rpm;        // 转速 (RPM)
    float mech_angle; // 机械角度 (rad, 0 ~ 2π)
    float elec_angle; // 电角度   (rad, 0 ~ 2π)  ← FOC 直接使用
} AS5600_DataTypeDef;

uint8_t as5600_init(void);
uint8_t as5600_update(void);
void as5600_get_data(AS5600_DataTypeDef *data);

#endif