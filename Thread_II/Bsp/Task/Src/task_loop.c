/**
 * @file task_loop.c
 * @author A-rtos (A-rtos@outlook.com)
 * @brief 循环任务函数
 * @version 0.1
 * @date 2026-01-31
 *
 * @copyright Copyright (c) 2026 A-rtos
 *
 */

#include "task_loop.h"
#include "drv_motor.h"
#include <stdio.h>

/**
 * @brief 循环任务函数
 */
void task_loop(void)
{
    /* ADC校准，启动常规组ADC转换，并开启DMA搬运 */
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)tMC.Sample.AdcBuff, 3);

    /* 设置初始占空比 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* 开启对应通道PWM输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* 开启TIM_CHANNEL_4中断 */
    HAL_TIM_Base_Start_IT(&htim1);

    /* 使能栅极驱动SD1/2/3/4 */
    HAL_GPIO_WritePin(SD1_GPIO_Port, SD1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD2_GPIO_Port, SD2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD3_GPIO_Port, SD3_Pin, GPIO_PIN_SET);

    printf("FOC Start...\r\n");

    tMC.Motor.RunState = MOTOR_SENSORUSE;
    tMC.Motor.RunMode = ENCODER_CALIB;

    while (1)
    {
        HAL_Delay(1000);
        led_b_toggle();
    }
}
