#ifndef __ADC_SIGNAL_H
#define __ADC_SIGNAL_H

#include <stdint.h>

#define ADC_VREF 3.3f             // ADC标准电压
#define ADC_MAX 4095.0f           // 12位ADC
#define CURRENT_SENSITIVITY 0.02f // 采样电阻阻值：0.004Ω × 三路相电流采样放⼤器的出⼚增益均为 5V = 电流采样灵敏度0.02f

#define VBUS_DIV_RATIO ((124.0f + 4.7f) / 4.7f)     //母线电压换算公式

//获得母线电压采样
float FOC_GetVbus(uint16_t adc_raw);
//获得母线电流采样
float FOC_GetCbus(uint16_t adc_raw);
//获得温度采样
float FOC_GetNTC(uint16_t adc_raw);

// 电流采样转换
float CurrentSense_Convert(uint16_t raw, float offset);


#endif