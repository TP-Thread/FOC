#ifndef ALGO_FOC_H
#define ALGO_FOC_H

#include "stdint.h"

#define SIN_RAD 0X0300
#define U0_90 0x0000
#define U90_180 0x0100
#define U180_270 0x0200
#define U270_360 0x0300

typedef struct
{
    float Iu; // U相电流
    float Iv; // V相电流
    float Iw; // W相电流

    float Ialpha; // α轴电流
    float Ibeta;  // β轴电流

    float SinVal; // 正弦值
    float CosVal; // 余弦值

    float Id;          // d轴电流
    float Iq;          // q轴电流
    float IdLPF;       // d轴电流滤波值
    float IqLPF;       // q轴电流滤波值
    float IdLPFFactor; // d轴电流滤波系数
    float IqLPFFactor; // q轴电流滤波系数

    float Ud; // d轴电压
    float Uq; // q轴电压

    float Ualpha; // α轴电压
    float Ubeta;  // β轴电压

    float Ubus; // 母线电压

    uint16_t PwmCycle;   // PWM周期
    uint16_t PwmLimit;   // 最大占空比
    uint16_t DutyCycleA; // A相占空比
    uint16_t DutyCycleB; // B相占空比
    uint16_t DutyCycleC; // C相占空比
} FOC_t;

void angle_calculate(int32_t angle, float *sinval, float *cosval);
void amplitude_limit(int32_t *input, int32_t min, int32_t max);

void clarke_transform(FOC_t *p);
void park_transform(FOC_t *p);
void park_inv_transform(FOC_t *p);
void us_limit(FOC_t *p);
void svpwm_calculate(FOC_t *p);

#endif /* ALGO_FOC_H */
