#ifndef ALGO_PID_H
#define ALGO_PID_H

typedef struct
{
    float Kp;        // 比例系数
    float Ki;        // 积分系数
    float Kd;        // 微分系数
    float Ref;       // 目标值
    float Fbk;       // 反馈值
    float Out;       // 输出值
    float Err;       // 本次误差
    float ErrLast;   // 上次误差
    float AllowErr;  // 允许误差
    float Integrate; // 积分项
    float OutMax;    // 输出上限
    float OutMin;    // 输出下限
    float KpMax;     // 比例系数上限（适用于分段式或模糊PID）
    float KpMin;     // 比例系数下限（适用于分段式或模糊PID）
} PID_t;

void pid_calculate(PID_t *p);
void pid_clear(PID_t *p);

#endif /* ALGO_PID_H */
