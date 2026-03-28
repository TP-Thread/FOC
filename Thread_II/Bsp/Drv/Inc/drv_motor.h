#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "main.h"
#include "algo_foc.h"
#include "algo_pid.h"

#define LOW_RESITOR 4.7f        // 母线电压检测下端电阻
#define HIGH_RESITOR 100.0f     // 母线电压检测上端电阻
#define SAMPLING_RESITOR 0.005f // 相电流采样电阻
#define MAGNIFICATION 10.0f     // 采样放大倍数
#define ADC_RESOLUTION 4096.0f  // ADC分辨率
#define ADC_VREF 3.3f           // ADC基准电压
#define PWM_LIMLT 7800          // 限制最大占空比
#define PWM_CYCLE 8500          // PWM周期占空比
#define TS 0.00005f             // FOC执行间隔

#define B_VALUE 3434.0f         // B值 (25℃/50℃)3380K  (25℃/85℃)3434K  (25℃/100℃)3455K
#define TEMP_REF 298.15f        // 参考温度值  25℃ + 273.15
#define RESISTOR_REF 10000.0f   // 参考温度下的阻值
#define RESISTOR_OTHER 10000.0f // 分压的阻值

#define VBUS_FACTOR ((ADC_VREF / ADC_RESOLUTION) / (LOW_RESITOR / (LOW_RESITOR + HIGH_RESITOR))) // 母线电压计算系数
#define CURRENT_FACTOR ((ADC_VREF / ADC_RESOLUTION) / MAGNIFICATION / SAMPLING_RESITOR)    // 相电流计算系数

/* 参数辨识状态 */
#define RESISTANCE_IDENTIFICATION 0X00 // 相电阻识别
#define INDUCTANCE_IDENTIFICATION 0X01 // 相电感识别

/* 运行状态 */
#define CURRENT_CALIB 0X00    // 相电流校准
#define MOTOR_STOP 0X01       // 停机
#define MOTOR_ERROR 0X02      // 故障报错
#define MOTOR_IDENTIFY 0X03   // 参数辨识
#define MOTOR_SENSORUSE 0X04  // 有感控制
#define MOTOR_SENSORLESS 0X05 // 无感控制

/* 有感运行模式 */
#define ENCODER_CALIB 0X00          // 编码器校准
#define CURRENT_OPEN_LOOP 0X01      // 电流开环
#define CURRENT_CLOSE_LOOP 0X02     // 电流闭环
#define SPEED_CURRENT_LOOP 0X03     // 速度闭环
#define POS_SPEED_CURRENT_LOOP 0X04 // 位置闭环

/* 无感运行模式 */
#define STRONG_DRAG_CURRENT_OPEN 0X05           // 电流开环强拖
#define STRONG_DRAG_CURRENT_CLOSE 0X06          // 电流闭环强拖
#define STRONG_DRAG_SMO_SPEED_CURRENT_LOOP 0X07 // 强拖切滑膜速度电流闭环
#define HFI_CURRENT_CLOSE 0X08                  // 电流闭环高频注入（测试HFI角度收敛效果）
#define HFI_SPEED_CURRENT_CLOSE 0X09            // 高频注入速度电流闭环
#define HFI_POS_SPEED_CURRENT_CLOSE 0X0A        // 高频注入位置速度电流闭环

#define HALF_PI 1.5707963f
#define ONE_PI 3.1415926f
#define TWO_PI 6.2831853f

#define CW 0  // 编码器顺时针方向
#define CCW 1 // 编码器逆时针方向

#define ENCODER_LINE 1024              // 编码器线数
#define PUL_MAX (4 * ENCODER_LINE - 1) // 单圈脉冲最大值

#define PUL_ANGLE_FACTOR (4095.0f / PUL_MAX) // 角度系数

#define POLEPAIRS 7    // 默认极对数
#define ACCELERATION 3 // 默认加速度

#define SPEED_DIVISION_FACTOR 2 // 速度环分频系数
#define POS_DIVISION_FACTOR 4   // 位置环分频系数

typedef struct
{
    uint8_t RunState; // 运行状态
    uint8_t RunMode;  // 运行模式
} Motor_t;

typedef struct
{
    uint8_t EndFlag;     // 计算完成标志
    uint16_t OffsetCnt;  // 基准值计算次数
    int32_t IuOffset;    // U相电流偏置值
    int32_t IvOffset;    // V相电流偏置值
    int32_t IwOffset;    // W相电流偏置值

    int32_t UdcRaw;      // 母线电压原始值
    int32_t IuRaw;       // U相电流原始值
    int32_t IvRaw;       // V相电流原始值
    int32_t IwRaw;       // W相电流原始值

    float UdcReal;       // 母线电压真实值
    float IuReal;        // U相电流真实值
    float IvReal;        // V相电流真实值
    float IwReal;        // W相电流真实值

    float BusFactor;     // 母线电压计算系数
    float CurrentFactor; // 相电流计算系数
    
    uint16_t AdcBuff[3]; // 用于ADC规则通道接收数据
} Sample_t;

typedef struct
{
    uint8_t Dir;                 // 编码器方向
    uint8_t Type;                // 编码器类型
    uint8_t PolePairs;           // 转子极对数
    int32_t EncoderVal;          // 编码器原始数据
    int32_t EncoderValMax;       // 编码器最大原始值
    int32_t ElectricalVal;       // 电气角度
    uint16_t CalibFlag;          // 校准完成标志
    uint16_t CalibOffset;        // 转子零位偏差（编码器零点与电角度零点的偏移量）

    uint8_t Index;               // Index脉冲检测标志

    float ElectricalSpdSet;      // 给定的电角速度
    float ElectricalValSet;      // 给定的电角度
} Encoder_t;

typedef struct
{
    uint16_t PosCalculateCnt;    // 位置计算计数
    uint16_t ElectricalValMax;   // 电角度最大值
    int32_t ElectricalPosThis;   // 本次电角位置
    int32_t ElectricalPosLast;   // 上次电角位置
    int32_t ElectricalPosChange; // 单位时间位移
    int32_t ElectricalPosSum;    // 绝对电角位置
    int32_t MechanicalPosRaw;    // 绝对机械位置
    int32_t MechanicalPosSet;    // 目标机械位置
} Position_t;

typedef struct
{
    int16_t CurrentSpeedDir;
    uint16_t SpeedCalculateCnt;  // 速度计算计数
    uint16_t ElectricalValMax;   // 电角度最大值
    int32_t ElectricalPosThis;   // 本次电角位置
    int32_t ElectricalPosLast;   // 上次电角位置
    int32_t ElectricalPosChange; // 单位时间位移
    float ElectricalSpeedFactor; // 电角速度系数
    float ElectricalSpeedRaw;    // 原始电角速度

    float ElectricalSpeedLPF;       // 原始电角速度滤波值
    float ElectricalSpeedLPFFactor; // 原始电角速度滤波系数

    float MechanicalSpeed;        // 滤波后机械速度
    float MechanicalSpeedSet;     // 目标机械速度
    float MechanicalSpeedSetLast; // 上次目标机械速度
} Speed_t;

typedef struct
{
    float StartSpeed;   // 初始速度
    float EndSpeed;     // 末速度
    float AccSpeed;     // 加速度
    float SumSpeed;     // 速度增量
    float DecSpeed;     // 减速度
    float SpeedOut;     // 输出目标速度
    uint8_t FinishFlag; // 加减速完成标志
} Tshape_t;

typedef struct
{
    Motor_t Motor;
    Sample_t Sample;
    Encoder_t Encoder;
    FOC_t Foc;
    PID_t PosPid;
    PID_t SpdPid;
    PID_t IqPid;
    PID_t IdPid;
    Position_t Position;
    Speed_t Speed;
    Tshape_t TAccDec;
} MotorCtrl_t;

extern MotorCtrl_t tMC;

void motor_init(void);
void motor_ctrl(void);

void sensoruse_ctrl(void);

void eangle_generator(Encoder_t *p);
void eangle_calculate(Encoder_t *p);

void position_calculate(Position_t *p);
void speed_calculate(Speed_t *p);
void tshape_acc_dec(Tshape_t *p);

void current_offset_calculate(Sample_t *p);
void current_calculate(Sample_t *p);
void voltage_calculate(Sample_t *p);

#endif /* DRV_MOTOR_H */
