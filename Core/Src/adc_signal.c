#include "adc_signal.h"

// 三相电流计算公式
float CurrentSense_Convert(uint16_t raw, float offset)
{
    return ((float)raw - offset) * ADC_VREF / ADC_MAX / CURRENT_SENSITIVITY;
}

// 获得母线电压采样
float FOC_GetVbus(uint16_t adc_raw)
{
    float adc_voltage = (float)adc_raw * ADC_VREF / ADC_MAX;
    float vbus = adc_voltage * VBUS_DIV_RATIO;
    return vbus;
}

// 获得母线电流采样
float FOC_GetCbus(uint16_t adc_raw)
{
    float adc_voltage = (float)adc_raw * ADC_VREF / ADC_MAX;
    float cbus = adc_voltage / (0.004 * 20);
    return cbus;
}

// 获得温度采样
float FOC_GetNTC(uint16_t adc_raw)
{
    // 1. ADC 原始值转换为电压
    float v_adc = (float)adc_raw * 3.3f / 4095.0f;

    // 2. 防止除零和无效阻值
    if (v_adc <= 0.0f || v_adc >= 3.3f)
        return -999.0f;

    // 3. 根据分压关系计算 NTC 阻值
    float r_ntc = 10000.0f * v_adc / (3.3f - v_adc);

    // 4. 使用 B 值公式计算温度
    float temp_k = 1.0f / (1.0f / 298.15f + logf(r_ntc / 10000.0f) / 3950.0f);

    float ntc = temp_k - 273.15f;

    // 5. 开尔文转换为摄氏度
    return ntc;
}

