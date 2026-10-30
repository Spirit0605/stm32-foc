#ifndef __FOC_H
#define __FOC_H

#include "math.h"
#include "main.h"

#define INV_SQRT3       0.57735026919f      //(1.0f / sqrtf(3.0f))
#define SQRT3_BY_2      0.86602540378f      
#define INV_SQRT3       0.57735026919f

typedef struct
{
    float kp;       
    float ki;       

    float integral;     //积分
    float out_min;  
    float out_max;  
} PI_Controller_t;      //保存 PI 控制器的状态

typedef struct
{
    float id_ref;     
    float iq_ref;

    float vd;
    float vq;

    float valpha;
    float vbeta;

    float duty_a;
    float duty_b;
    float duty_c;
} FOC_Control_t;        //保存整条控制链的输入、输出

//Clark变换
void FOC_Clarke(float ia, float ib, float *ialpha, float *ibeta);
//Park变换
void FOC_Park(float ialpha, float ibeta, float theta, float *id, float *iq);

/*
    作用：初始化PI控制器
    pi	要初始化的 PI 控制器
    kp	比例系数
    ki	积分系数
    out_min	输出下限
    out_max	输出上限    
*/
void PI_Init(PI_Controller_t *pi, float kp, float ki, float out_min, float out_max);
/*
    作用：调电流
    pi:         PI 控制器及其内部状态
    ref:        目标电流
    feedback:   实际测量电流
    dt:         实际控制周期，单位秒
*/
float PI_Update(PI_Controller_t *pi, float ref, float feedback, float dt);
/*
    逆 Park 坐标变换
    作用：把旋转坐标系中的电压 \(V_d,V_q\)，转换成静止坐标系中的电压 \(V_\alpha,V_\beta\)。
    vd、vq：PI 控制器输出的两轴电压。
    theta：转子电角度，单位是弧度。
    valpha、vbeta：转换后的静止坐标系电压
*/
void FOC_InversePark(float vd, float vq, float theta, float *valpha, float *vbeta);
/*
    空间矢量脉宽调制
    valpha、vbeta	目标电压矢量
    vbus	        实际直流母线电压，单位 V
    duty_a/b/c	    A、B、C 三相占空比，通常为 0～1
*/
void FOC_SVPWM(float valpha, float vbeta, float vbus, float *duty_a, float *duty_b, float *duty_c);

#endif