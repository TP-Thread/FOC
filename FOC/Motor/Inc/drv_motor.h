/**
 * @file drv_motor.h
 * @author dufuqi (dufuqi@roborock.com)
 * @brief 电机驱动相关
 * @version 0.1
 * @date 2026-04-01
 * 
 * @copyright Copyright (c) 2026 roborock
 * 
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "motor_ctrl.h"

#define ENCODER_COUNT_NUM               (16383U)
#define ENCODER_MECH_STEP               (MOTOR_PI_2 / ENCODER_COUNT_NUM)
#define ENCODER_ELEC_STEP               (MOTOR_PI_2 * POLE_PAIRS / ENCODER_COUNT_NUM)
#define ENCODER_SPEED_FACTOR            (60.0f / (float)POLE_PAIRS / 6.28f)

#define CURRENT_FACTOR                  (3.3f / 4096) / (0.012f * 6.8f) /* 电流计算因子 = ADC换算电压值/电流采样电阻/电流放大倍数 */

void motor_init(motor_t* motor);

void motor_start(void);
void motor_stop(void);
void motor_brake(void);

void motor_angle_speed_get(motor_t *motor);
void motor_joint_zero_set(motor_t *motor);
void motor_current_get(motor_t* motor);
void motor_pwm_set(motor_t* motor);

#endif /* DRV_MOTOR_H */
