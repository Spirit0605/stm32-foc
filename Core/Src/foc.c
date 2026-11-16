#include "foc.h"

// void FOC_Clarke(float32_t ia, float32_t ib, float32_t *ialpha, float32_t *ibeta)
// {
//     if (ialpha == NULL || ibeta == NULL)
//         return;

//     *ialpha = ia;
//     *ibeta = INV_SQRT3 * (ia + 2.0f * ib); // 1/√3
// }

// void FOC_Park(float32_t ialpha, float32_t ibeta, float32_t theta, float32_t *id, float32_t *iq)
// {
//     float32_t cos_t = arm_cos_f32(theta);
//     float32_t sin_t = arm_sin_f32(theta);

//     // float32_t cos_t = cosf(theta);
//     // float32_t sin_t = sinf(theta);

//     *id = ialpha * cos_t + ibeta * sin_t;
//     *iq = -ialpha * sin_t + ibeta * cos_t;
// }

// 优化后的限幅内联函数（0跳转，单周期硬件执行）
static __inline float32_t clampf(float32_t x, float32_t min, float32_t max)
{
    x = (x > max) ? max : x;
    x = (x < min) ? min : x;
    return x;
}

// PI 控制器初始化
void PI_Init(PI_Controller_t *pi, float32_t kp, float32_t ki, float32_t out_min, float32_t out_max)
{
    if (pi == 0)
        return;

    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
}

// 离散PI，带积分限幅 调电流
// float32_t PI_Update(PI_Controller_t *pi, float32_t ref, float32_t feedback, float32_t dt)
// {
//     if (pi == 0 || dt <= 0.0f)
//         return 0.0f;

//     //1.计算误差
//     float32_t error = ref - feedback;

//     //2.计算新的积分累加值
//     float32_t integral_new = pi->integral + pi->ki * error * dt;

//     //3.抗积分饱和限幅
//     integral_new = clampf(integral_new, pi->out_min, pi->out_max);

//     //4.计算总控制输出
//     float32_t output = pi->kp * error + integral_new;

//     //5.总输出二次限幅
//     output = clampf(output, pi->out_min, pi->out_max);

//     //6.动态更新控制器状态
//     pi->integral = integral_new;

//     return output;
// }

// 离散 PI：条件积分抗饱和
float32_t PI_Update(PI_Controller_t *pi, float32_t ref, float32_t feedback, float32_t dt)
{
    if (pi == NULL || dt <= 0.0f)
        return 0.0f;

    // 1. 计算误差
    float32_t error = ref - feedback;

    // 2. 比例项
    float32_t proportional = pi->kp * error;

    // 3. 计算候选积分项
    float32_t integral_new = pi->integral + pi->ki * error * dt;

    // 4. 计算未限幅输出
    float32_t output_unlimited = proportional + integral_new;

    // 5. 条件积分：输出饱和且误差继续推动饱和时，停止积分
    if ((output_unlimited > pi->out_max && error > 0.0f) || (output_unlimited < pi->out_min && error < 0.0f))
    {
        integral_new = pi->integral;
    }

    // 6. 更新积分状态
    pi->integral = clampf(integral_new, pi->out_min, pi->out_max);

    // 7. 计算最终输出
    float32_t output = proportional + pi->integral;

    // 8. 输出限幅
    return clampf(output, pi->out_min, pi->out_max);
}

/* 反 Park：dq 坐标系 -> alpha-beta 静止坐标系
 * theta 使用弧度
 */
// void FOC_InversePark(float32_t vd, float32_t vq, float32_t theta, float32_t *valpha, float32_t *vbeta)
// {
//     if (valpha == 0 || vbeta == 0)
//         return;

//     float32_t sin_theta = arm_sin_f32(theta);
//     float32_t cos_theta = arm_cos_f32(theta);

//     // float32_t sin_theta = sinf(theta);
//     // float32_t cos_theta = cosf(theta);

//     *valpha = vd * cos_theta - vq * sin_theta;
//     *vbeta = vd * sin_theta + vq * cos_theta;
// }

// SVPWM：alpha-beta 电压 -> 三相占空比
// vbus：直流母线电压，单位 V
// duty：范围 0.0 ~ 1.0
// 生成三相占空比
void FOC_SVPWM(float32_t valpha, float32_t vbeta, float32_t vbus, float32_t *duty_a, float32_t *duty_b, float32_t *duty_c)
{
    if (duty_a == 0 || duty_b == 0 || duty_c == 0 || vbus <= 0.1f)
        return;

    /* 计算三相等效电压 */
    float32_t ua = valpha;
    float32_t ub = -0.5f * valpha + SQRT3_BY_2 * vbeta;
    float32_t uc = -0.5f * valpha - SQRT3_BY_2 * vbeta;

    /* 求最大、最小相电压 */
    float32_t umax = ua;
    float32_t umin = ua;

    if (ub > umax)
        umax = ub;
    if (uc > umax)
        umax = uc;

    if (ub < umin)
        umin = ub;
    if (uc < umin)
        umin = uc;

    /* 过调制保护：缩放电压矢量，避免占空比越界 */
    float32_t span = umax - umin;

    if (span > vbus)
    {
        float32_t scale = vbus / span;

        ua *= scale;
        ub *= scale;
        uc *= scale;

        umax *= scale;
        umin *= scale;
    }

    /* 注入零序电压，实现中心对齐 SVPWM */
    float32_t offset = -0.5f * (umax + umin);

    *duty_a = clampf(0.5f + (ua + offset) / vbus, 0.0f, 1.0f);

    *duty_b = clampf(0.5f + (ub + offset) / vbus, 0.0f, 1.0f);

    *duty_c = clampf(0.5f + (uc + offset) / vbus, 0.0f, 1.0f);
}
