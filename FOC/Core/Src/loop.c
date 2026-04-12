/**
 * @file loop.c
 * @author dufuqi (dufuqi@roborock.com)
 * @brief 电机初始化和运行状态控制
 * @version 0.1
 * @date 2026-04-01
 * 
 * @copyright Copyright (c) 2026 roborock
 * 
 */

#include "drv_motor.h"
#include "motor_protect.h"

#include "gd32f30x.h"
#include "systick.h"      /* delay function */
#include <stdio.h>        /* printf function */

float id_target = 0.0f;       /* 设定d轴电流参考值 */
float iq_target = 0.0f;       /* 设定q轴电流参考值 */
float speed_target = 0.0f;    /* 设定速度参考值 */
float position_target = 0.0f; /* 设定位置参考值 */

/* 寿命测试序列 */
static const float position_sequence[4] = {0.0f, 2.5f, 0.0f, -2.5f};
static uint8_t position_index = 0;

/* 堵转测试序列 */
static const float lock_test_sequence[4] = {0.0f, -2.5f, -5.0f, -2.5f};
static const uint16_t lock_test_hold_ms[4] = {3000U, 1000U, 5000U, 1000U};
static uint8_t lock_test_index = 0;

/**
 * @brief 主循环
 * 
 */
void loop(void)
{
    motor_init(&motor);
    motor_protect_peripheral_init();
    motor_protect_var_init();

    /* DRV8328要求nSLEEP引脚拉高15m后启动电机 */
    gpio_bit_set(GPIOB, GPIO_PIN_0);
    delay_1ms(15);
    motor_start();

    /* 电角度零位补偿角 */
    motor.angle_offset = (*(uint16_t *)FMC_OFFSET_ANGLE_ADDR);

    /* 开机将当前输出轴位置定义为软件零位 */
    motor_joint_zero_set(&motor);

    /* 进入校准模式 */
    // motor.mode = MC_MODE_CALIB;
    // motor.angle_mode = CUSTOM_MODE;

    /* 进入位置闭环模式 */
    motor.mode = MC_MODE_POSITION;

    while (1)
    {
        /* MT6816原始数据读取 */
        // printf("CS1 (PA15): raw = %d\r\n", encoder.angle_raw);
        // printf("motor.angle_elec = %f\r\n", motor.angle_elec);
        // printf("motor.angle_offset = %d\r\n", motor.angle_offset);

        // printf("motor.pwm.tA = %d\r\n", motor.pwm.tA);
        // printf("motor.pwm.tB = %d\r\n", motor.pwm.tB);
        // printf("motor.pwm.tC = %d\r\n", motor.pwm.tC);

        // printf("motor.ud = %f\r\n", motor.ud);
        // printf("motor.uq = %f\r\n", motor.uq);

        /* 设定dq轴电流参考值 */
        // motor.id_ref = id_target;
        // motor.iq_ref = iq_target;

        /* 设定速度参考值 */
        // motor.speed_ref = speed_target;

        /* 设定位置参考值 */
        // motor.position_ref = position_target;

        /* 寿命测试 */
        position_target = position_sequence[position_index];
        motor.position_ref = position_target;
        delay_1ms(4000);
        position_index++;
        if (position_index >= 4)
        {
            position_index = 0;
        }

        /* 堵转测试 */
        // motor.position_ref = lock_test_sequence[lock_test_index];
        // delay_1ms(lock_test_hold_ms[lock_test_index]);
        // lock_test_index++;
        // if (lock_test_index >= 4)
        // {
        //     lock_test_index = 0;
        // }

        /* 读取nFAULT引脚状态 */
        uint8_t nfault = gpio_input_bit_get(GPIOA, GPIO_PIN_12);
        if (nfault == 0)
        {
            printf("nFAULT pin is low, fault occurs!\r\n");
        }

        if (FAULT_NONE != motor.fault)
        {
            printf("motor fault code: %d\r\n", motor.fault);
            motor_stop();
        }
    }
}
