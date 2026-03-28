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

/* Private variables ---------------------------------------------------------*/
extern volatile uint16_t LedTaskTim;
extern volatile uint16_t LcdTaskTim;
extern volatile uint16_t KeyTaskTim;
extern volatile uint16_t UsartTaskTim;

/* Private functions ---------------------------------------------------------*/
/**
 * @brief TIM3编码器Index中断回调函数
 * @note 当编码器Z脉冲到来时触发此中断，TIM3->CNT会自动清零
 */
void HAL_TIMEx_EncoderIndexCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        tMC.Encoder.Index = 1; // 设置Index脉冲检测标志
        // printf("TIM3->CNT = %d\r\n", TIM3->CNT); // 打印当前计数值，验证Index功能
        // led_r_toggle(); // 调试用LED指示
    }
}

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
    tMC.Sample.IuRaw = ADC2->JDR1;            // 获取U相电流
    tMC.Sample.IwRaw = ADC2->JDR2;            // 获取W相电流
    tMC.Sample.UdcRaw = tMC.Sample.AdcBuff[0]; // 获取母线电压
    tMC.Encoder.EncoderVal = TIM3->CNT;       // 获取编码器值

    motor_ctrl(); // 电机控制

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tMC.Foc.DutyCycleA); // 更新PWMA比较值
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, tMC.Foc.DutyCycleB); // 更新PWMB比较值
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, tMC.Foc.DutyCycleC); // 更新PWMC比较值
}
