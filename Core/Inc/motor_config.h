/**
  ******************************************************************************
  * @file    motor_config.h
  * @brief   整个电机项目的"单一事实来源"：硬件引脚、电机参数、限幅。
  *
  *          改硬件只改这一个文件。
  *
  *  硬件：STM32F407ZGT6 (HSE 8MHz -> 168MHz)
  *        + POWER_DRV8323HR_V1.01 (DRV8323HR, 六路 PWM, 0.004R 低边采样, 增益 5V/V)
  *        + 2804 无刷电机 (12N14P, 极对数 7)
  *        + AS5600 磁编码器 (I2C 0x36)
  ******************************************************************************
  */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "main.h"

/* ==========================================================================
 * 1. 电机本体参数（来自《规格书2804电机A0版.pdf》）
 * ========================================================================== */
#define MOTOR_POLE_PAIRS        7U          /* 12N14P -> 极对数 7            */
#define MOTOR_RS_OHM            2.55f       /* 相电阻                        */
#define MOTOR_LS_HENRY          0.00086f    /* 相电感                        */
#define MOTOR_KV_RPM_PER_V      220.0f      /* KV                            */
#define MOTOR_FLUX_WB           0.0035f     /* 磁链                          */
#define MOTOR_I_RATED_A         0.5f        /* 额定电流                      */
#define MOTOR_I_MAX_A           2.0f        /* 最大电流                      */

/* ==========================================================================
 * 2. 供电与限幅
 *    母线电压未最终确定时先按 12V；换了电源只改 MOTOR_VBUS_NOMINAL。
 *    安全限压必须 <= VBUS/1.732（否则线性调制区不够，波形会削顶）。
 * ========================================================================== */
#define MOTOR_VBUS_NOMINAL      12.0f       /* 母线电压 [V]                  */
#define MOTOR_VBUS_MIN          10.0f       /* 欠压保护阈值 [V]              */
#define MOTOR_VBUS_MAX          16.0f       /* 过压保护阈值 [V]              */

/* 首版调试用的电压限幅。2804 相电阻只有 2.55R，12V 直接堵转约 4.7A，
 * 远超电机 2A 上限，所以限幅必须给得很小。
 *   3.0V  -> 最坏情况堵转电流约 1.2A
 *   6.0V  -> 最坏情况堵转电流约 2.4A（必须配合电源限流）           */
#define MOTOR_VOLTAGE_LIMIT     3.0f        /* [V] 相电压矢量幅值上限        */
#define MOTOR_VELOCITY_LIMIT    104.7f      /* [rad/s] 约 1000 RPM，位置模式用*/

/* ==========================================================================
 * 3. 驱动板 POWER_DRV8323HR_V1.01
 * ========================================================================== */
/* PWM：TIM8 中心对齐，APB2 定时器时钟 168MHz
 *   20kHz 中心对齐 -> ARR = 168e6/(2*20000) = 4200
 * 驱动板允许 10k~40kHz。 */
#define DRV_PWM_FREQ_HZ         20000U
#define DRV_TIM8_CLK_HZ         168000000U
#define DRV_PWM_ARR             ((DRV_TIM8_CLK_HZ / (2U * DRV_PWM_FREQ_HZ)) - 1U)   /* 4199 */

/* 死区。DRV8323HR 是栅极驱动 + 外部 MOS，六路 PWM 模式下死区由 MCU 负责。
 * 500ns 对绝大多数 MOS 都安全。若示波器看到直通或波形变差再调整。 */
#define DRV_DEADTIME_NS         500U
/* DTG 寄存器编码：tDTS = 1/168MHz ≈ 5.952ns，500ns / 5.952ns ≈ 84 (0..127 线性区) */
#define DRV_DEADTIME_DTG        ((uint32_t)((DRV_DEADTIME_NS * (DRV_TIM8_CLK_HZ / 1000000U)) / 1000U))

/* 电流采样（来自说明书第 6 章"电气参数与出厂配置"）
 *   R36/R37/R38 = 0.004R，放大器增益 5 V/V
 *   -> 20 mV/A。2804 满量程 2A 只有 40mV，约 50 个 ADC 码，偏粗。
 * 零偏（0A 时的 ADC 读数）在启动时实测，不用理论值。 */
#define DRV_SHUNT_OHM           0.004f
#define DRV_CSA_GAIN_V_PER_V    5.0f
#define DRV_I_SENSE_V_PER_A     (DRV_SHUNT_OHM * DRV_CSA_GAIN_V_PER_V)   /* 0.02 V/A */

/* 母线采样（说明书 4.3 / 4.4 节） */
#define DRV_CBUS_V_PER_A        0.08f       /* ADC_CBUS = Ibus * 0.08 V/A    */
#define DRV_VBUS_DIV_RATIO      0.0365f     /* ADC_VBUS = VCC * 0.0365       */

/* ==========================================================================
 * 4. 编码器 AS5600
 * ========================================================================== */
#define AS5600_I2C_ADDR7        0x36U
#define AS5600_RESOLUTION       4096.0f     /* 12 bit                        */
/* 采样周期：TIM6 1kHz，即在 1ms 中断里读一次角度。
 * 2804 跑到 1000RPM 时电频率 = 1000/60*7 = 117Hz，1kHz 采样 = 每电周期 8.6 点，
 * 靠角度外推补偿。要跑满 2600RPM 需把 I2C 提到 1MHz 或换 SSI 编码器。 */
#define AS5600_SAMPLE_HZ        1000U
#define AS5600_DIR              1           /* +1 / -1，装反了改这里        */

/* ==========================================================================
 * 5. 闭环参数初值（第 5 阶段调参的起点，不是最终值）
 * ========================================================================== */
#define CTRL_SPEED_KP_DEFAULT   0.05f
#define CTRL_SPEED_KI_DEFAULT   0.5f
#define CTRL_SPEED_LPF_TC       0.010f      /* 速度低通时间常数 [s]         */
#define CTRL_RAMP_RPM_PER_S     1000.0f     /* 斜坡：每秒最多变化多少 RPM    */
#define CTRL_TARGET_RPM_MAX     1000.0f     /* 目标转速上限                 */

/* ==========================================================================
 * 6. 安全与上电顺序
 * ========================================================================== */
/* DRV_EN 拉高后要等 DRV8323H 唤醒，说明书要求至少 1ms */
#define DRV_EN_WAKEUP_MS        5U
/* FAULT 判定：拉高后连续多少次轮询为高才认为"无故障" */
#define DRV_FAULT_STABLE_CNT    5U

#endif /* MOTOR_CONFIG_H */
