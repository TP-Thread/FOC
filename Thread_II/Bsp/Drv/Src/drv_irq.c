/**
 * @file drv_irq.c
 * @author A-rtos (A-rtos@outlook.com)
 * @brief 中断回调函数
 * @version 0.1
 * @date 2026-01-30
 *
 * @copyright Copyright (c) 2026 A-rtos
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_irq.h"
#include "drv_motor.h"
#include "drv_kth78.h"

/* Private variables ---------------------------------------------------------*/
extern volatile uint16_t LedTaskTim;
extern volatile uint16_t LcdTaskTim;
extern volatile uint16_t KeyTaskTim;
extern volatile uint16_t UsartTaskTim;

/* Private functions ---------------------------------------------------------*/
/**
 * @brief 定时器中断回调函数
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim1.Instance) // 20KHz触发频率
    {
        HAL_ADCEx_InjectedStart_IT(&hadc2); // 启动注入组ADC转换，并开启转换完成中断
    }

    if (htim->Instance == htim2.Instance) // 10KHz触发频率
    {
        LedTaskTim++;   // LED任务计时
        LcdTaskTim++;   // LCD任务计时
        KeyTaskTim++;   // 按键扫描任务计时
        UsartTaskTim++; // 串口任务计时
    }
}

/**
 * @brief 注入组ADC转换完成中断回调函数
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    tMC.Sample.IuRaw = ADC2->JDR1;                    /* 获取U相电流 */
    tMC.Sample.IwRaw = ADC2->JDR2;                    /* 获取W相电流 */
    tMC.Sample.UdcRaw = tMC.Sample.AdcBuff[0];        /* 获取母线电压 */
    tMC.Encoder.EncoderVal = kth78_read_angle() / 16; /* 获取编码器值 */

    /* 电机FOC控制*/
    motor_ctrl();

    /* 更新PWM比较值 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tMC.Foc.DutyCycleA);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, tMC.Foc.DutyCycleB);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, tMC.Foc.DutyCycleC);
}
