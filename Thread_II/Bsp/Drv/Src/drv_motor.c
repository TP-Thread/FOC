/**
 * @file drv_motor.c
 * @author A-rtos (A-rtos@outlook.com)
 * @brief 电机驱动相关函数
 * @version 0.1 搭建工程
 * @version 0.2 基于Index脉冲的机械零点对齐校准编码器
 * @date 2026-01-30
 *
 * @copyright Copyright (c) 2026 A-rtos
 *
 */

#include "drv_motor.h"

MotorCtrl_t tMC;

/**
 * @brief 电机系统参数初始化
 */
void motor_init(void)
{
    tMC.Motor.RunState = CURRENT_CALIB; // 设置电机最初的运行状态
    tMC.Motor.RunMode = ENCODER_CALIB;  // 设置电机最初的运行模式

    tMC.Sample.CurrentFactor = CURRENT_FACTOR; // 相电流计算系数(由采样电阻值和放大倍数以及ADC分辨率计算得出)
    tMC.Sample.BusFactor = VBUS_FACTOR;        // 母线电压计算系数（由分压电阻计算得出）

    tMC.Encoder.Dir = CCW;               // 设置编码器的方向（逆时针转动 角度从0向360度增加）
    tMC.Encoder.PolePairs = POLEPAIRS;   // 设置电机的极对数（磁铁数除以2）
    tMC.Encoder.EncoderValMax = PUL_MAX; // 设置编码器单圈脉冲的最大值4095

    tMC.Encoder.Index = 0; // Index脉冲检测标志清零

    tMC.Foc.IdLPFFactor = 0.1f;   // 设置d轴电流低通滤波系数
    tMC.Foc.IqLPFFactor = 0.1f;   // 设置q轴电流低通滤波系数
    tMC.Foc.Ubus = 24;            // 设置母线电压
    tMC.Foc.PwmCycle = PWM_CYCLE; // 设置PWM周期
    tMC.Foc.PwmLimit = PWM_LIMLT; // 设置PWM限幅值

    tMC.Position.ElectricalValMax = PUL_MAX; // 设置编码器单圈脉冲的最大值

    tMC.TAccDec.AccSpeed = ACCELERATION; // 设置速度模式下的加速度

    tMC.Speed.ElectricalValMax = PUL_MAX;       // 设置编码器单圈脉冲的最大值
    tMC.Speed.ElectricalSpeedLPFFactor = 0.05f; // 设置速度低通滤波系数
    tMC.Speed.ElectricalSpeedFactor = 146.5f;   // 设置速度计算系数

    tMC.IqPid.Kp = 0.2f;   // 设置q轴PID比例系数
    tMC.IqPid.Ki = 0.002f; // 设置q轴PID比例系数
    tMC.IqPid.OutMax =  24 * 0.57735f;  // 设置q轴PID输出上限初始值为Ubus/√3，限制Uq
    tMC.IqPid.OutMin = -24 * 0.57735f;

    tMC.IdPid.Kp = 0.2f;   // 设置d轴PID比例系数
    tMC.IdPid.Ki = 0.002f; // 设置d轴PID比例系数
    tMC.IdPid.OutMax =  24 * 0.57735f;  // 设置d轴PID输出上限初始值为Ubus/√3，限制Ud
    tMC.IdPid.OutMin = -24 * 0.57735f;

    tMC.SpdPid.Kp = 0.001f;    // 设置速度PID比例系数
    tMC.SpdPid.KpMax = 0.005f; // 设置速度PID比例系数最大值（用于分段或模糊PID）
    tMC.SpdPid.KpMin = 0.001f; // 设置速度PID比例系数最小值（用于分段或模糊PID）
    tMC.SpdPid.Ki = 0.000002f; // 设置速度PID积分系数
    tMC.SpdPid.OutMax = 8;     // 设置速度PID输出上限,4006电机适配40A电调，IqPid.Ref限制为单相最大电流/√3
    tMC.SpdPid.OutMin = -8;    // 设置速度PID输出下限

    tMC.PosPid.Kp = 0.5f;       // 设置位置PID比例系数
    tMC.PosPid.Ki = 0;          // 设置位置PID积分系数
    tMC.PosPid.Kd = 0;          // 设置位置PID微分系数
    tMC.PosPid.OutMax = 14000;  // 设置位置PID输出上限
    tMC.PosPid.OutMin = -14000; // 设置位置PID输出下限
}

/********************************************************************************
 * 相电流和母线电压计算
 *******************************************************************************/
/**
 * @brief 获取相电流和总线电压基准值
 */
void current_offset_calculate(Sample_t *p)
{
    if (p->OffsetCnt == 0)
    {
        p->OffsetCnt = 0;

        p->EndFlag = 0;
        p->IuOffset = 0;
        p->IwOffset = 0;
    }

    if (p->OffsetCnt < 1024) // 采集1024次数据求平均值
    {
        p->OffsetCnt++;

        p->IuOffset += p->IuRaw;
        p->IwOffset += p->IwRaw;
    }
    else
    {
        p->OffsetCnt = 0;
        p->EndFlag = 1; // 相电流校准完成标志MC.Sample.EndFlag

        p->IuOffset = p->IuOffset >> 10; // U相电流偏置值
        p->IwOffset = p->IwOffset >> 10; // W相电流偏置值
    }
}

/**
 * @brief 计算三相电流值，以流入中性点的方向为正，下桥臂采样需反向
 */
void current_calculate(Sample_t *p)
{
    p->IuReal = -(p->IuOffset - p->IuRaw) * p->CurrentFactor;
    p->IwReal = -(p->IwOffset - p->IwRaw) * p->CurrentFactor;
    p->IvReal = -p->IuReal - p->IwReal;
}

/**
 * @brief 计算母线电压值
 */
void voltage_calculate(Sample_t *p)
{
    p->UdcReal = p->UdcRaw * p->BusFactor; // 母线电压真实值tMC.Sample.UdcReal
}

/********************************************************************************
 * 电角度计算
 *******************************************************************************/
/**
 * @brief 电角度发生器，根据设定的电角速度速度生成电角度
 */
void eangle_generator(Encoder_t *p)
{
    p->ElectricalValSet += (0.00005f * p->ElectricalSpdSet * 0.01666f * p->EncoderValMax); // 根据设定速度 计算电角度值
    if (p->ElectricalValSet >= p->EncoderValMax)                                           // 越过编码器边界点
    {
        p->ElectricalValSet = p->ElectricalValSet - p->EncoderValMax;
    }

    if (p->ElectricalValSet < 0) // 越过编码器边界点
    {
        p->ElectricalValSet = p->ElectricalValSet + p->EncoderValMax;
    }
}

/**
 * @brief 将编码器数据转换为电角度
 */
void eangle_calculate(Encoder_t *p)
{
    if (p->Dir == 1) // 判断编码器方向
    {
        p->EncoderVal = p->EncoderValMax - p->EncoderVal; // 方向取反
    }

    /* 将编码器的机械角度转换为FOC控制所需的电角度 */
    p->ElectricalVal = ((p->EncoderVal - p->CalibOffset) * p->PolePairs) % p->EncoderValMax;

    if (p->ElectricalVal < 0) // 处理校准可能带来的负值
    {
        p->ElectricalVal = p->ElectricalVal + p->EncoderValMax;
    }
}

/********************************************************************************
 * 位置和速度计算
 *******************************************************************************/
/**
 * @brief 计算位置
 */
void position_calculate(Position_t *p)
{
    p->ElectricalPosChange = p->ElectricalPosThis - p->ElectricalPosLast; // 计算单位时间内位移
    p->ElectricalPosLast = p->ElectricalPosThis;

    if (p->ElectricalPosChange >= (p->ElectricalValMax * 0.5f)) // 越过编码器零点
    {
        p->ElectricalPosChange = p->ElectricalPosChange - p->ElectricalValMax;
    }

    if (p->ElectricalPosChange <= (-p->ElectricalValMax * 0.5f)) // 越过编码器零点
    {
        p->ElectricalPosChange = p->ElectricalPosChange + p->ElectricalValMax;
    }

    p->ElectricalPosSum = p->ElectricalPosSum + p->ElectricalPosChange; // 计算总位置
}

/**
 * @brief 计算速度
 */
void speed_calculate(Speed_t *p)
{
    p->ElectricalPosChange = p->ElectricalPosThis - p->ElectricalPosLast; // 计算单位时间内位移
    p->ElectricalPosLast = p->ElectricalPosThis;

    if (p->ElectricalPosChange >= (p->ElectricalValMax * 0.5f)) // 越过编码器零点
    {
        p->ElectricalPosChange = p->ElectricalPosChange - p->ElectricalValMax;
    }

    if (p->ElectricalPosChange <= (-p->ElectricalValMax * 0.5f))
    {
        p->ElectricalPosChange = p->ElectricalPosChange + p->ElectricalValMax; // 越过编码器零点
    }

    p->ElectricalSpeedRaw = p->ElectricalPosChange * p->ElectricalSpeedFactor; // 计算原始电角速度
}

/**
 * @brief T形加减速算法，用于生成平滑的速度轨迹，避免速度突变导致的电流冲击和机械振动
 */
void tshape_acc_dec(Tshape_t *p)
{
    if (p->FinishFlag == 0)
    {
        if ((p->EndSpeed - p->StartSpeed) > 0)
        {
            p->SumSpeed = p->SumSpeed + p->AccSpeed;   // 累加速度
            p->SpeedOut = p->StartSpeed + p->SumSpeed; // 计算输出速度

            if (p->SpeedOut >= p->EndSpeed)
            {
                p->SpeedOut = p->EndSpeed; // 限幅
                p->FinishFlag = 1;         // 达到目标速度，标记完成
            }
        }

        if ((p->EndSpeed - p->StartSpeed) < 0)
        {
            p->SumSpeed = p->SumSpeed - p->AccSpeed;   // 递减速度
            p->SpeedOut = p->StartSpeed + p->SumSpeed; // 计算输出速度

            if (p->SpeedOut <= p->EndSpeed)
            {
                p->SpeedOut = p->EndSpeed;
                p->FinishFlag = 1; // 达到目标速度，标记完成
            }
        }

        if (p->FinishFlag == 1)
        {
            p->SumSpeed = 0;
        }
    }
}

/********************************************************************************
 * 电机控制
 *******************************************************************************/
/**
 * @brief 电机控制，更新电机运行参数
 */
void motor_ctrl()
{
    if (tMC.Sample.EndFlag == 1) // 相电流校准后执行
    {
        current_calculate(&tMC.Sample); // 计算三相电流值
        voltage_calculate(&tMC.Sample); // 计算母线电压值

        if (tMC.Sample.UdcReal <= 10 || tMC.Sample.UdcReal >= 40)
        {
            tMC.Motor.RunState = MOTOR_ERROR; // 供电不正常
        }
    }

    /* 状态机 */
    switch (tMC.Motor.RunState)
    {
    case CURRENT_CALIB: // 相电流校准
    {
        current_offset_calculate(&tMC.Sample);
        if (tMC.Sample.EndFlag == 1)
        {
            tMC.Motor.RunState = MOTOR_SENSORUSE;
        }
    }
    break;
    case MOTOR_SENSORUSE: // 有感控制
    {
        sensoruse_ctrl();
    }
    break;
    case MOTOR_ERROR:
    {
        tMC.Foc.DutyCycleA = 0;
        tMC.Foc.DutyCycleB = 0;
        tMC.Foc.DutyCycleC = 0;
    }
    break;
    case MOTOR_STOP:
    {
        tMC.Foc.DutyCycleA = 0;
        tMC.Foc.DutyCycleB = 0;
        tMC.Foc.DutyCycleC = 0;
    }
    break;
    default:
        break;
    }
}

/**
 * @brief 有感控制
 */
void sensoruse_ctrl(void)
{
    /* 计算电角度 */
    eangle_calculate(&tMC.Encoder);
    angle_calculate(tMC.Encoder.ElectricalVal, &tMC.Foc.SinVal, &tMC.Foc.CosVal);

    /* 状态机 */
    switch (tMC.Motor.RunMode)
    {
    case ENCODER_CALIB: // 编码器校准（基于Index脉冲的机械零点对齐）
    {
        if (tMC.Encoder.CalibFlag == 0) // 第一阶段：定位到90度
        {
            tMC.Foc.Ud += 0.0001f; // 缓慢增加d轴电压
            tMC.Foc.Uq = 0;        // q轴电压为0
            tMC.Foc.SinVal = 1;    // 强制电角度90°
            tMC.Foc.CosVal = 0;

            if (tMC.Foc.Ud >= 1)
            {
                tMC.Foc.Ud = 0;
                tMC.Encoder.Index = 0;
                tMC.Encoder.CalibFlag = 1;
            }
        }

        if (tMC.Encoder.CalibFlag == 1) // 第二阶段：缓慢旋转寻找机械零点
        {
            tMC.Foc.Uq = 0.5f;                                                                      // 施加q轴电压
            tMC.Encoder.ElectricalSpdSet = 50;                                                      // 设置较低的开环转速
            eangle_generator(&tMC.Encoder);                                                         // 生成递增的电角度
            angle_calculate((int32_t)tMC.Encoder.ElectricalValSet, &tMC.Foc.SinVal, &tMC.Foc.CosVal); // 计算sin/cos值

            if (tMC.Encoder.Index) // 检测Index脉冲
            {
                tMC.Foc.Uq = 0;
                tMC.Encoder.ElectricalSpdSet = 0;
                tMC.Encoder.ElectricalValSet = 0;
                tMC.Encoder.CalibFlag = 2;
            }
        }

        if (tMC.Encoder.CalibFlag == 2) // 第三阶段：定位到电角度0度
        {
            tMC.Foc.Ud += 0.0001f;
            tMC.Foc.Uq = 0;
            tMC.Foc.SinVal = 0; // 强制电角度0°
            tMC.Foc.CosVal = 1;

            if (tMC.Foc.Ud >= 1)
            {
                tMC.Foc.Ud = 0;
                tMC.Encoder.CalibFlag = 0;
                tMC.Encoder.CalibOffset = tMC.Encoder.EncoderVal; // 记录编码器零点与电角度零点的偏移量

                tMC.Motor.RunMode = POS_SPEED_CURRENT_LOOP; // 完成转子校准，进入控制模式

                /* 初始化位置累加器为0 */
                tMC.Position.ElectricalPosSum = 0;
                tMC.Position.ElectricalPosLast = tMC.Encoder.ElectricalVal;
            }
        }

        park_inv_transform(&tMC.Foc);
    }
    break;
    case CURRENT_OPEN_LOOP: // 电流开环
    {
        park_inv_transform(&tMC.Foc); // 给定Uq值电机转动
    }
    break;
    case CURRENT_CLOSE_LOOP: // 电流闭环
    {
        tMC.IqPid.Ref = tMC.Sample.AdcBuff[1] * 0.002f; // 使用波轮电位器给电机目标电流Iq_ref电机转动
        tMC.IdPid.Ref = 0;

        /* 电流环 */
        tMC.Foc.Iu = tMC.Sample.IuReal;
        tMC.Foc.Iv = tMC.Sample.IvReal;
        clarke_transform(&tMC.Foc);                                                               // Iu,Iv → Iα,Iβ
        park_transform(&tMC.Foc);                                                                 // Iα,Iβ → Id,Iq
        tMC.Foc.IdLPF = tMC.Foc.Id * tMC.Foc.IdLPFFactor + tMC.Foc.IdLPF * (1 - tMC.Foc.IdLPFFactor); // Id低通滤波
        tMC.Foc.IqLPF = tMC.Foc.Iq * tMC.Foc.IqLPFFactor + tMC.Foc.IqLPF * (1 - tMC.Foc.IqLPFFactor); // Iq低通滤波
        tMC.IqPid.Fbk = tMC.Foc.IqLPF;
        tMC.IdPid.Fbk = tMC.Foc.IdLPF;
        pid_calculate(&tMC.IqPid); // 输出Uq
        pid_calculate(&tMC.IdPid); // 输出Ud
        tMC.Foc.Uq = tMC.IqPid.Out;
        tMC.Foc.Ud = tMC.IdPid.Out;
        us_limit(&tMC.Foc);
        park_inv_transform(&tMC.Foc); // Ud,Uq → Uα,Uβ
    }
    break;
    case SPEED_CURRENT_LOOP: // 速度闭环+电流闭环
    {
        tMC.Speed.SpeedCalculateCnt++;
        tMC.Speed.MechanicalSpeedSet = (tMC.Sample.AdcBuff[1] > 50 ? tMC.Sample.AdcBuff[1] : 0); // 使用波轮电位器给电机目标转速

        /* 速度环 */
        if (tMC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)
        {
            tMC.Speed.SpeedCalculateCnt = 0;
            tMC.Speed.ElectricalPosThis = tMC.Encoder.ElectricalVal; // 获取当前电角度
            speed_calculate(&tMC.Speed);                            // 根据当前电角度和上次电角度计算电角速度
            tMC.Speed.ElectricalSpeedLPF = tMC.Speed.ElectricalSpeedRaw * tMC.Speed.ElectricalSpeedLPFFactor + tMC.Speed.ElectricalSpeedLPF * (1 - tMC.Speed.ElectricalSpeedLPFFactor);
            tMC.Speed.MechanicalSpeed = tMC.Speed.ElectricalSpeedLPF / tMC.Encoder.PolePairs; // 转换为机械速度

            if (tMC.Speed.MechanicalSpeedSet != tMC.Speed.MechanicalSpeedSetLast) // 给定了新的目标速度
            {
                tMC.TAccDec.StartSpeed = tMC.Speed.MechanicalSpeedSetLast * tMC.Encoder.PolePairs; // 设置初速度
                tMC.TAccDec.EndSpeed = tMC.Speed.MechanicalSpeedSet * tMC.Encoder.PolePairs;       // 设置末速度

                tshape_acc_dec(&tMC.TAccDec); // T形加减速计算

                if (tMC.TAccDec.FinishFlag == 1) // 执行完加减速
                {
                    tMC.Speed.MechanicalSpeedSetLast = tMC.Speed.MechanicalSpeedSet; // 更新上次目标速度
                    tMC.TAccDec.FinishFlag = 0;
                }
            }

            tMC.SpdPid.Ref = tMC.TAccDec.SpeedOut;         // 获得目标值
            tMC.SpdPid.Fbk = tMC.Speed.ElectricalSpeedLPF; // 反馈速度值

            if (tMC.SpdPid.Fbk > -2000 && tMC.SpdPid.Fbk < 2000)
            {
                tMC.SpdPid.Kp = tMC.SpdPid.KpMax; // 低速区，提高增益，增强响应
            }
            else
            {
                tMC.SpdPid.Kp = tMC.SpdPid.KpMin; // 高速区，降低增益，防止超调
            }

            pid_calculate(&tMC.SpdPid);    // 速度闭环PID计算
            tMC.IqPid.Ref = tMC.SpdPid.Out; // 速度环输出作为电流环输入
        }

        /* 电流环 */
        tMC.Foc.Iu = tMC.Sample.IuReal;
        tMC.Foc.Iv = tMC.Sample.IvReal;
        clarke_transform(&tMC.Foc);
        park_transform(&tMC.Foc);
        tMC.Foc.IdLPF = tMC.Foc.Id * tMC.Foc.IdLPFFactor + tMC.Foc.IdLPF * (1 - tMC.Foc.IdLPFFactor);
        tMC.Foc.IqLPF = tMC.Foc.Iq * tMC.Foc.IqLPFFactor + tMC.Foc.IqLPF * (1 - tMC.Foc.IqLPFFactor);
        tMC.IqPid.Fbk = tMC.Foc.IqLPF;
        tMC.IdPid.Fbk = tMC.Foc.IdLPF;
        pid_calculate(&tMC.IqPid);
        pid_calculate(&tMC.IdPid);
        tMC.Foc.Uq = tMC.IqPid.Out;
        tMC.Foc.Ud = tMC.IdPid.Out;
        us_limit(&tMC.Foc);
        park_inv_transform(&tMC.Foc);
    }
    break;
    case POS_SPEED_CURRENT_LOOP: // 位置闭环+速度闭环+电流闭环
    {
        tMC.Position.PosCalculateCnt++;
        tMC.Speed.SpeedCalculateCnt++;
        tMC.Position.MechanicalPosSet = -tMC.Sample.AdcBuff[1]; // 使用波轮电位器给电机目标位置

        /* 位置环 */
        if (tMC.Position.PosCalculateCnt >= POS_DIVISION_FACTOR)
        {
            tMC.Position.PosCalculateCnt = 0;
            tMC.Position.ElectricalPosThis = tMC.Encoder.ElectricalVal; // 获取当前位置
            position_calculate(&tMC.Position);                         // 计算总位置
            tMC.PosPid.Fbk = tMC.Position.ElectricalPosSum;             // 反馈实际位置
            tMC.PosPid.Ref = tMC.Position.MechanicalPosSet * POLEPAIRS; // 给定目标位置
            tMC.Position.MechanicalPosRaw = tMC.Position.ElectricalPosSum / POLEPAIRS;
            pid_calculate(&tMC.PosPid); // 位置闭环PID计算
        }

        /* 速度环 */
        if (tMC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)
        {
            tMC.Speed.SpeedCalculateCnt = 0;
            tMC.Speed.ElectricalPosThis = tMC.Encoder.ElectricalVal;
            speed_calculate(&tMC.Speed); // 计算速度
            tMC.Speed.ElectricalSpeedLPF = tMC.Speed.ElectricalSpeedRaw * tMC.Speed.ElectricalSpeedLPFFactor + tMC.Speed.ElectricalSpeedLPF * (1 - tMC.Speed.ElectricalSpeedLPFFactor);
            tMC.Speed.MechanicalSpeed = tMC.Speed.ElectricalSpeedLPF / POLEPAIRS;
            tMC.SpdPid.Ref = tMC.PosPid.Out;               // 给定速度值
            tMC.SpdPid.Fbk = tMC.Speed.ElectricalSpeedLPF; // 反馈速度值

            if (tMC.SpdPid.Fbk > -2000 && tMC.SpdPid.Fbk < 2000)
            {
                tMC.SpdPid.Kp = tMC.SpdPid.KpMax;
            }
            else
            {
                tMC.SpdPid.Kp = tMC.SpdPid.KpMin;
            }

            pid_calculate(&tMC.SpdPid); // 速度闭环PID计算
            tMC.IqPid.Ref = tMC.SpdPid.Out;
        }

        /* 电流环 */
        tMC.Foc.Iu = tMC.Sample.IuReal;
        tMC.Foc.Iv = tMC.Sample.IvReal;
        clarke_transform(&tMC.Foc);
        park_transform(&tMC.Foc);
        tMC.Foc.IdLPF = tMC.Foc.Id * tMC.Foc.IdLPFFactor + tMC.Foc.IdLPF * (1 - tMC.Foc.IdLPFFactor);
        tMC.Foc.IqLPF = tMC.Foc.Iq * tMC.Foc.IqLPFFactor + tMC.Foc.IqLPF * (1 - tMC.Foc.IqLPFFactor);
        tMC.IqPid.Fbk = tMC.Foc.IqLPF;
        tMC.IdPid.Fbk = tMC.Foc.IdLPF;
        pid_calculate(&tMC.IqPid);
        pid_calculate(&tMC.IdPid);
        tMC.Foc.Uq = tMC.IqPid.Out;
        tMC.Foc.Ud = tMC.IdPid.Out;
        us_limit(&tMC.Foc);
        park_inv_transform(&tMC.Foc);
    }
    break;
    }

    /* SVPWM调制 */
    svpwm_calculate(&tMC.Foc);        // 生成三相PWM占空比
}
