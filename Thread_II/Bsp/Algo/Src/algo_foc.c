/**
 * @file algo_foc.c
 * @author A-rtos (A-rtos@outlook.com)
 * @brief FOC 算法实现
 * @version 0.1
 * @date 2026-01-30
 *
 * @copyright Copyright (c) 2026 A-rtos
 *
 */

#include "algo_foc.h"
#include <math.h>

/* 正余弦表Q15格式 */
const uint16_t u16SinTable[256] = {
    0x0000, 0x00C9, 0x0192, 0x025B, 0x0324, 0x03ED, 0x04B6, 0x057F, 0x0648, 0x0711, 0x07D9, 0x08A2, 0x096A, 0x0A33, 0x0AFB, 0x0BC4,
    0x0C8C, 0x0D54, 0x0E1C, 0x0EE3, 0x0FAB, 0x1072, 0x113A, 0x1201, 0x12C8, 0x138F, 0x1455, 0x151C, 0x15E2, 0x16A8, 0x176E, 0x1833,
    0x18F9, 0x19BE, 0x1A82, 0x1B47, 0x1C0B, 0x1CCF, 0x1D93, 0x1E57, 0x1F1A, 0x1FDD, 0x209F, 0x2161, 0x2223, 0x22E5, 0x23A6, 0x2467,
    0x2528, 0x25E8, 0x26A8, 0x2767, 0x2826, 0x28E5, 0x29A3, 0x2A61, 0x2B1F, 0x2BDC, 0x2C99, 0x2D55, 0x2E11, 0x2ECC, 0x2F87, 0x3041,
    0x30FB, 0x31B5, 0x326E, 0x3326, 0x33DF, 0x3496, 0x354D, 0x3604, 0x36BA, 0x376F, 0x3824, 0x38D9, 0x398C, 0x3A40, 0x3AF2, 0x3BA5,
    0x3C56, 0x3D07, 0x3DB8, 0x3E68, 0x3F17, 0x3FC5, 0x4073, 0x4121, 0x41CE, 0x427A, 0x4325, 0x43D0, 0x447A, 0x4524, 0x45CD, 0x4675,
    0x471C, 0x47C3, 0x4869, 0x490F, 0x49B4, 0x4A58, 0x4AFB, 0x4B9D, 0x4C3F, 0x4CE0, 0x4D81, 0x4E20, 0x4EBF, 0x4F5D, 0x4FFB, 0x5097,
    0x5133, 0x51CE, 0x5268, 0x5302, 0x539B, 0x5432, 0x54C9, 0x5560, 0x55F5, 0x568A, 0x571D, 0x57B0, 0x5842, 0x58D3, 0x5964, 0x59F3,
    0x5A82, 0x5B0F, 0x5B9C, 0x5C28, 0x5CB3, 0x5D3E, 0x5DC7, 0x5E4F, 0x5ED7, 0x5F5D, 0x5FE3, 0x6068, 0x60EB, 0x616E, 0x61F0, 0x6271,
    0x62F1, 0x6370, 0x63EE, 0x646C, 0x64E8, 0x6563, 0x65DD, 0x6656, 0x66CF, 0x6746, 0x67BC, 0x6832, 0x68A6, 0x6919, 0x698B, 0x69FD,
    0x6A6D, 0x6ADC, 0x6B4A, 0x6BB7, 0x6C23, 0x6C8E, 0x6CF8, 0x6D61, 0x6DC9, 0x6E30, 0x6E96, 0x6EFB, 0x6F5E, 0x6FC1, 0x7022, 0x7083,
    0x70E2, 0x7140, 0x719D, 0x71F9, 0x7254, 0x72AE, 0x7307, 0x735E, 0x73B5, 0x740A, 0x745F, 0x74B2, 0x7504, 0x7555, 0x75A5, 0x75F3,
    0x7641, 0x768D, 0x76D8, 0x7722, 0x776B, 0x77B3, 0x77FA, 0x783F, 0x7884, 0x78C7, 0x7909, 0x794A, 0x7989, 0x79C8, 0x7A05, 0x7A41,
    0x7A7C, 0x7AB6, 0x7AEE, 0x7B26, 0x7B5C, 0x7B91, 0x7BC5, 0x7BF8, 0x7C29, 0x7C59, 0x7C88, 0x7CB6, 0x7CE3, 0x7D0E, 0x7D39, 0x7D62,
    0x7D89, 0x7DB0, 0x7DD5, 0x7DFA, 0x7E1D, 0x7E3E, 0x7E5F, 0x7E7E, 0x7E9C, 0x7EB9, 0x7ED5, 0x7EEF, 0x7F09, 0x7F21, 0x7F37, 0x7F4D,
    0x7F61, 0x7F74, 0x7F86, 0x7F97, 0x7FA6, 0x7FB4, 0x7FC1, 0x7FCD, 0x7FD8, 0x7FE1, 0x7FE9, 0x7FF0, 0x7FF5, 0x7FF9, 0x7FFD, 0x7FFE};

/**
 * @brief 计算指定角度的正余弦值
 *
 * @param angle 输入0-4095 对应0-360度
 * @param sinval
 * @param cosval
 */
void angle_calculate(int32_t angle, float *sinval, float *cosval)
{
    amplitude_limit(&angle, 0, 4095);

    angle = (int32_t)(angle * 0.25f); // 0-4095转换为0-1023 与数组大小对应

    switch (angle & SIN_RAD)
    {
    case U0_90:
        *sinval = u16SinTable[(uint8_t)(angle)];
        *cosval = u16SinTable[(uint8_t)(0xFF - (uint8_t)(angle))];
        break;
    case U90_180:
        *sinval = u16SinTable[(uint8_t)(0xFF - (uint8_t)(angle))];
        *cosval = -u16SinTable[(uint8_t)(angle)];
        break;
    case U180_270:
        *sinval = -u16SinTable[(uint8_t)(angle)];
        *cosval = -u16SinTable[(uint8_t)(0xFF - (uint8_t)(angle))];
        break;
    case U270_360:
        *sinval = -u16SinTable[(uint8_t)(0xFF - (uint8_t)(angle))];
        *cosval = u16SinTable[(uint8_t)(angle)];
        break;
    default:
        break;
    }

    *sinval = *sinval * 0.0000305f; // 还原正弦值
    *cosval = *cosval * 0.0000305f; // 还原余弦值
}

/**
 * @brief 限幅函数
 */
void amplitude_limit(int32_t *input, int32_t min, int32_t max)
{
    if (*input <= min)
    {
        *input = min;
    }

    if (*input >= max)
    {
        *input = max;
    }
}

/**
 * @brief Clarke恒幅值变换
 */
void clarke_transform(FOC_t *p)
{
    p->Ialpha = p->Iu;
    p->Ibeta = (p->Iu * 0.57735027f) + (p->Iv * 1.1547004f);
}

/**
 * @brief Park变换
 */
void park_transform(FOC_t *p)
{
    p->Id = (p->Ialpha * p->CosVal) + (p->Ibeta * p->SinVal);
    p->Iq = (-p->Ialpha * p->SinVal) + (p->Ibeta * p->CosVal);
}

/**
 * @brief 反Park变换
 */
void park_inv_transform(FOC_t *p)
{
    p->Ualpha = p->Ud * p->CosVal - p->Uq * p->SinVal;
    p->Ubeta = p->Uq * p->CosVal + p->Ud * p->SinVal;
}

/**
 * @brief Ud,Uq合成矢量幅值限制Ubus/√3
 * 
 * @param p FOC控制结构体指针
 */
void us_limit(FOC_t *p)
{
    float Umax;             // 母线电压的√3分之一
    float UdSq, UqSq;       // Ud²和Uq²
    float UdUqSumSq;        // Ud²+Uq²
    float UmaxSq;           // Umax²
    float temp;             // 临时变量
    
    /* 计算电压限制值：Umax = Ubus / √3 */
    Umax = p->Ubus * 0.57735f;
    UmaxSq = Umax * Umax;
    
    /* 计算d轴和q轴电压的平方 */
    UdSq = p->Ud * p->Ud;
    UqSq = p->Uq * p->Uq;
    UdUqSumSq = UdSq + UqSq;
    
    /* 判断是否超过限制 */
    if (UdUqSumSq > UmaxSq)
    {
        temp = UmaxSq - UdSq;
        p->Uq = (p->Uq >= 0) ? sqrtf(temp) : -sqrtf(temp);
    }
}

/**
 * @brief 合成空间电压矢量
 */
void svpwm_calculate(FOC_t *p)
{
    float U1, U2, U3 = 0;
    float X, Y, Z = 0;
    float T1, T2, T1Temp, T2Temp = 0;
    uint8_t A, B, C, N = 0;
    uint16_t Ta, Tb, Tc = 0;

    U1 = p->Ubeta;
    U2 = (0.866f * p->Ualpha) - (0.5f * p->Ubeta);
    U3 = (-0.866f * p->Ualpha) - (0.5f * p->Ubeta);

    if (U1 > 0)
    {
        A = 1;
    }
    else
    {
        A = 0;
    }
    if (U2 > 0)
    {
        B = 1;
    }
    else
    {
        B = 0;
    }
    if (U3 > 0)
    {
        C = 1;
    }
    else
    {
        C = 0;
    }
    N = 4 * C + 2 * B + A;

    X = (1.732f * p->PwmCycle * p->Ubeta) / p->Ubus;
    Y = (1.5f * p->Ualpha * p->PwmCycle + 0.866f * p->Ubeta * p->PwmCycle) / p->Ubus;
    Z = (-1.5f * p->Ualpha * p->PwmCycle + 0.866f * p->Ubeta * p->PwmCycle) / p->Ubus;

    switch (N)
    {
    case 3:
    {
        T1 = -Z;
        T2 = X;
    }
    break;
    case 1:
    {
        T1 = Z;
        T2 = Y;
    }
    break;
    case 5:
    {
        T1 = X;
        T2 = -Y;
    }
    break;
    case 4:
    {
        T1 = -X;
        T2 = Z;
    }
    break;
    case 6:
    {
        T1 = -Y;
        T2 = -Z;
    }
    break;
    case 2:
    {
        T1 = Y;
        T2 = -X;
    }
    break;
    default:
    {
        T1 = 0;
        T2 = 0;
    }
    break;
    }

    T1Temp = T1;
    T2Temp = T2;
    if (T1 + T2 > p->PwmLimit) // 当T1+T2超过PWM限制时，按比例缩放防止调制度过高导致波形失真
    {
        T1 = p->PwmLimit * T1Temp / (T1Temp + T2Temp);
        T2 = p->PwmLimit * T2Temp / (T1Temp + T2Temp);
    }

    /* 七段式SVPWM时间分配 */
    Ta = (uint16_t)((p->PwmCycle - T1 - T2) * 0.25f);
    Tb = (uint16_t)(Ta + T1 * 0.5f);
    Tc = (uint16_t)(Tb + T2 * 0.5f);

    switch (N)
    {
    case 3:
    {
        p->DutyCycleA = Ta;
        p->DutyCycleB = Tb;
        p->DutyCycleC = Tc;
    }
    break;
    case 1:
    {
        p->DutyCycleA = Tb;
        p->DutyCycleB = Ta;
        p->DutyCycleC = Tc;
    }
    break;
    case 5:
    {
        p->DutyCycleA = Tc;
        p->DutyCycleB = Ta;
        p->DutyCycleC = Tb;
    }
    break;
    case 4:
    {
        p->DutyCycleA = Tc;
        p->DutyCycleB = Tb;
        p->DutyCycleC = Ta;
    }
    break;
    case 6:
    {
        p->DutyCycleA = Tb;
        p->DutyCycleB = Tc;
        p->DutyCycleC = Ta;
    }
    break;
    case 2:
    {
        p->DutyCycleA = Ta;
        p->DutyCycleB = Tc;
        p->DutyCycleC = Tb;
    }
    break;
    default:
    {
        p->DutyCycleA = Ta;
        p->DutyCycleB = Tb;
        p->DutyCycleC = Tc;
    }
    break;
    }
}
