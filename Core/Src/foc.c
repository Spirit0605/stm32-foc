#include "foc.h"

void FOC_Clarke(float ia, float ib, float *ialpha, float *ibeta)
{
    if (ialpha == NULL || ibeta == NULL)
        return;

    *ialpha = ia;
    *ibeta = INV_SQRT3 * (ia + 2.0f * ib); // 1/√3
}

void FOC_Park(float ialpha, float ibeta, float theta, float *id, float *iq)
{
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    *id = ialpha * cos_t + ibeta * sin_t;
    *iq = -ialpha * sin_t + ibeta * cos_t;
}

// 限幅函数
static float clampf(float x, float min, float max)
{
    if (x > max)
        return max;
    if (x < min)
        return min;
    return x;
}

// PI 控制器初始化
void PI_Init(PI_Controller_t *pi, float kp, float ki, float out_min, float out_max)
{
    if (pi == 0)
        return;

    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
}

// 离散 PI，带积分限幅 调电流
float PI_Update(PI_Controller_t *pi, float ref, float feedback, float dt)
{
    if (pi == 0 || dt <= 0.0f)
        return 0.0f;

    float error = ref - feedback;

    float integral_new = pi->integral + pi->ki * error * dt;

    integral_new = clampf(integral_new, pi->out_min, pi->out_max);

    float output = pi->kp * error + integral_new;

    output = clampf(output, pi->out_min, pi->out_max);

    pi->integral = integral_new;

    return output;
}

/* 反 Park：dq 坐标系 -> alpha-beta 静止坐标系
 * theta 使用弧度
 */
void FOC_InversePark(float vd, float vq, float theta, float *valpha, float *vbeta)
{
    if (valpha == 0 || vbeta == 0)
        return;

    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    *valpha = vd * cos_theta - vq * sin_theta;
    *vbeta = vd * sin_theta + vq * cos_theta;
}

// SVPWM：alpha-beta 电压 -> 三相占空比
// vbus：直流母线电压，单位 V
// duty：范围 0.0 ~ 1.0
// 生成三相占空比
void FOC_SVPWM(float valpha, float vbeta, float vbus, float *duty_a, float *duty_b, float *duty_c)
{
    if (duty_a == 0 || duty_b == 0 || duty_c == 0 || vbus <= 0.1f)
        return;

    /* 计算三相等效电压 */
    float ua = valpha;
    float ub = -0.5f * valpha + SQRT3_BY_2 * vbeta;
    float uc = -0.5f * valpha - SQRT3_BY_2 * vbeta;

    /* 求最大、最小相电压 */
    float umax = ua;
    float umin = ua;

    if (ub > umax)
        umax = ub;
    if (uc > umax)
        umax = uc;

    if (ub < umin)
        umin = ub;
    if (uc < umin)
        umin = uc;

    /* 过调制保护：缩放电压矢量，避免占空比越界 */
    float span = umax - umin;

    if (span > vbus)
    {
        float scale = vbus / span;

        ua *= scale;
        ub *= scale;
        uc *= scale;

        umax *= scale;
        umin *= scale;
    }

    /* 注入零序电压，实现中心对齐 SVPWM */
    float offset = -0.5f * (umax + umin);

    *duty_a = clampf(0.5f + (ua + offset) / vbus, 0.0f, 1.0f);

    *duty_b = clampf(0.5f + (ub + offset) / vbus, 0.0f, 1.0f);

    *duty_c = clampf(0.5f + (uc + offset) / vbus, 0.0f, 1.0f);
}
