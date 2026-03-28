/**
 * @file algo_pid.c
 * @author A-rtos (A-rtos@outlook.com)
 * @brief PID 算法实现
 * @version 0.1
 * @date 2026-01-30
 *
 * @copyright Copyright (c) 2026 A-rtos
 *
 */

#include "algo_pid.h"

/**
 * @brief 抗积分饱和PID计算
 */
void pid_calculate(PID_t *p)
{
    p->Err = p->Ref - p->Fbk; // 计算误差

    if (p->Out < p->OutMax && p->Out > p->OutMin) // 没达到输出限幅就一直积分
    {
        p->Integrate = p->Integrate + p->Err;
    }

    p->Out = p->Kp * p->Err +               // 计算比例项
             p->Ki * p->Integrate +         // 计算积分项
             p->Kd * (p->Err - p->ErrLast); // 计算微分项

    if (p->Out >= p->OutMax) // 输出上限幅
    {
        p->Out = p->OutMax;
    }
    if (p->Out <= p->OutMin) // 输出下限幅
    {
        p->Out = p->OutMin;
    }

    p->ErrLast = p->Err;
}

/**
 * @brief 重置所有相关PID变量
 */
void pid_clear(PID_t *p)
{
    p->Ref = 0;       // 重置目标值
    p->Fbk = 0;       // 重置反馈值
    p->Out = 0;       // 重置输出值
    p->Err = 0;       // 重置误差值
    p->ErrLast = 0;   // 重置误差值
    p->Integrate = 0; // 重置积分项
}
