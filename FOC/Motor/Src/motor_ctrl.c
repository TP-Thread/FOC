/**
 * @file motor_ctrl.c
 * @author dufuqi (dufuqi@roborock.com)
 * @brief 电机控制相关函数实现
 * @version 0.1
 * @date 2026-04-01
 * 
 * @copyright Copyright (c) 2026 roborock
 * 
 */

#include "main.h"
#include "motor_ctrl.h"
#include "drv_motor.h"
#include "motor_protect.h"

/* motor control */
motor_t motor = {
    .state = MC_STATE_RUNNING,
    .mode = MC_MODE_CALIB,
    .angle_mode = ENCODER_MODE,

    .pole_pairs = POLE_PAIRS,
    .us_base = 16.0f,            /* 基本电压矢量幅值,设定为常值即可 */
    .us_out_max = 16.0f * 0.86f, /* 内切圆幅值比，略小于√3/2 */
    
    /* pid struct for flux control */
    .pid_flux.kp = CURRENT_LOOP_KP,
    .pid_flux.ki = CURRENT_LOOP_KI,
    .pid_flux.kd = 0,
    .pid_flux.integral_sum_upper_limit = CURRENT_LOOP_SUM_MAX,
    .pid_flux.integral_sum_lower_limit = CURRENT_LOOP_SUM_MIN,
    .pid_flux.output_upper_limit = CURRENT_LOOP_UOTPUT_MAX,
    .pid_flux.output_lower_limit = CURRENT_LOOP_UOTPUT_MIN,

    /* pid struct for torque control */
    .pid_torque.kp = CURRENT_LOOP_KP,
    .pid_torque.ki = CURRENT_LOOP_KI,
    .pid_torque.kd = 0,
    .pid_torque.integral_sum_upper_limit = CURRENT_LOOP_SUM_MAX,
    .pid_torque.integral_sum_lower_limit = CURRENT_LOOP_SUM_MIN,
    .pid_torque.output_upper_limit = CURRENT_LOOP_UOTPUT_MAX,
    .pid_torque.output_lower_limit = CURRENT_LOOP_UOTPUT_MIN,

    /* pid struct for speed control */
    .pid_speed.kp = SPEED_LOOP_KP,
    .pid_speed.ki = SPEED_LOOP_KI,
    .pid_speed.kd = 0,
    .pid_speed.integral_sum_upper_limit = SPEED_LOOP_SUM_MAX,
    .pid_speed.integral_sum_lower_limit = SPEED_LOOP_SUM_MIN,
    .pid_speed.output_upper_limit = SPEED_LOOP_UOTPUT_MAX,
    .pid_speed.output_lower_limit = SPEED_LOOP_UOTPUT_MIN,

    /* pid struct for position control */
    .pid_position.kp = POSITION_LOOP_KP,
    .pid_position.ki = 0,
    .pid_position.kd = 0,
    .pid_position.integral_sum_upper_limit = POSITION_LOOP_SUM_MAX,
    .pid_position.integral_sum_lower_limit = POSITION_LOOP_SUM_MIN,
    .pid_position.output_upper_limit = POSITION_LOOP_UOTPUT_MAX,
    .pid_position.output_lower_limit = POSITION_LOOP_UOTPUT_MIN,

    /* PWM struct */
    .pwm.pwm_top = PWM_COUNT,
    .pwm.tA = 0,
    .pwm.tB = 0,
    .pwm.tC = 0
};

/**
 * @brief 擦除FMC中存储偏移角度的页，为写入新数据做准备
 * 
 */
static void offset_angle_fmc_erase_pages(void)
{
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    fmc_page_erase(FMC_OFFSET_ANGLE_ADDR);
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    fmc_lock();
}

/**
 * @brief 将偏移角度数据写入FMC
 * 
 */
static void offset_angle_fmc_data_program(motor_t *motor)
{
    fmc_unlock();
    fmc_halfword_program(FMC_OFFSET_ANGLE_ADDR, motor->angle_offset);
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    fmc_lock();
}

/**
 * @brief 过调制处理，优先保证d轴电流环，调整q轴电压幅值
 * 
 */
static uint8_t over_modulation(motor_t *motor)
{
    uint8_t retval = 0;
    float mag;
    float temp, square_ud, square_uq;

    square_ud = motor->ud * motor->ud;
    square_uq = motor->uq * motor->uq;

    /* 过调制处理优先保证𝑑轴电流环，而对𝑞轴电流环缩放 */
    if (motor->us_out_max * motor->us_out_max - square_ud - square_uq < 0)
    {
        temp = motor->us_out_max * motor->us_out_max - square_ud;

        if (temp <= 0) /* ud超限，限制ud到最大值，uq强制为0 */
        {
            temp = 0;

            if (motor->ud > 0)
            {
                motor->ud = motor->us_out_max;
            }
            else
            {
                motor->ud = -motor->us_out_max;
            }

            motor->uq = 0;
        }
        else /* ud未超限，调整uq幅值 */
        {
            mag = sqrt_float(temp);
            if (motor->uq > 0)
            {
                motor->uq = mag;
            }
            else
            {
                motor->uq = -mag;
            }
        }

        retval = 1;
    }

    return retval;
}

/**
 * @brief 速度参考值加减速斜坡处理
 * 
 */
static float speed_ref_ramp(float target, float current)
{
    float delta;

    delta = target - current;

    if (delta > SPEED_RAMP_STEP)
    {
        return current + SPEED_RAMP_STEP;
    }
    if (delta < -SPEED_RAMP_STEP)
    {
        return current - SPEED_RAMP_STEP;
    }

    return target;
}

/**
 * @brief 将位置误差归一化到[-π, π]
 *
 */
static float position_error_normalize(float ref, float fbk)
{
    float error = ref - fbk;

    if (error > MOTOR_PI)
    {
        error -= MOTOR_PI_2;
    }

    if (error < -MOTOR_PI)
    {
        error += MOTOR_PI_2;
    }

    return error;
}

/**
 * @brief 位置参考轨迹规划（每周期限幅步进）
 *
 */
static float position_ref_plan_step(float target, float current)
{
    float delta;

    delta = position_error_normalize(target, current);

    if (delta > POSITION_TRAJ_STEP)
    {
        return current + POSITION_TRAJ_STEP;
    }
    if (delta < -POSITION_TRAJ_STEP)
    {
        return current - POSITION_TRAJ_STEP;
    }

    return target;
}

/**
 * @brief 电机控制，更新电机运行参数
 * 
 */
void motor_ctrl(void)
{
    triangle_struct phase_sincos;

    static uint8_t u8CalibFlag = 0;
    static uint8_t u8CalibCnt = 0;
    static uint8_t u8SpdCnt = 0;
    static uint8_t u8PosCnt = 0;
    static float f64PosError = 0;

    /* 获取三相电流值 */
    motor_current_get(&motor);
    // motor_protect_phase_loss_check();

    /* 获取电角度/电角速度/机械速度 */
    motor_angle_speed_get(&motor);
    sin_cos_float(motor.angle_elec, &phase_sincos.sin, &phase_sincos.cos);

    switch (motor.mode)
    {
    case MC_MODE_OPEN: /* 电流开环模式，直接控制d轴和q轴电压 */
        motor.ud = 0;
        motor.uq = 0.1f;
        rev_park(&motor.ualpha, &motor.ubeta, motor.ud, motor.uq, phase_sincos);
        break;
    case MC_MODE_CALIB: /* 计算编码器零点与电角度零点的偏移量 */
        /* 第一阶段：缓慢旋转寻找机械零点 */
        if (u8CalibFlag == 0) 
        {
            /* 施加q轴电压 */
            motor.ud = 0;
            motor.uq = 0.1f;

            /* 定位到机械角度接近0度 */
            if (motor.angle_raw > 0 && motor.angle_raw < 10)
            {
                u8CalibCnt++;
                if (u8CalibCnt >= 2)
                {
                    motor.uq = 0;
                    u8CalibCnt = 0;

                    u8CalibFlag = 1;
                }
            }
            else
            {
                u8CalibCnt = 0;
            }
        }

        /* 第二阶段：定位到电角度0度 */
        if (u8CalibFlag == 1)
        {
            motor.ud += 0.0001f;
            motor.uq = 0;

            /* 强制电角度0° */
            phase_sincos.sin = 0;
            phase_sincos.cos = 1;

            if (motor.ud >= 1)
            {
                motor.ud = 0;
                u8CalibFlag = 0;

                /* 记录电角度零点对应的编码器原始数据写入FMC */
                motor.angle_offset = motor.angle_raw;
                offset_angle_fmc_erase_pages();
                offset_angle_fmc_data_program(&motor);

                motor_stop(); /* 校准完成，停止电机 */
            }
        }

        rev_park(&motor.ualpha, &motor.ubeta, motor.ud, motor.uq, phase_sincos);
        break;
    case MC_MODE_CURRENT:
        /* 强制电角度0° */
        phase_sincos.sin = 0;
        phase_sincos.cos = 1;
        
        /* 电流环16KHz */
        clarke_amplitude(&motor.ialpha, &motor.ibeta, motor.ia, motor.ib);
        park(&motor.id, &motor.iq, motor.ialpha, motor.ibeta, phase_sincos);
        UTILS_LP_FAST(motor.id_lpf, motor.id, 0.1f);
        UTILS_LP_FAST(motor.iq_lpf, motor.iq, 0.1f);
        motor.ud = pid_regulation(motor.id_ref, motor.id, &motor.pid_flux);
        motor.uq = pid_regulation(motor.iq_ref, motor.iq, &motor.pid_torque);
        over_modulation(&motor);
        rev_park(&motor.ualpha, &motor.ubeta, motor.ud, motor.uq, phase_sincos);
        break;
    case MC_MODE_SPEED:
        /* 速度环2KHz */
        u8SpdCnt++;
        if(u8SpdCnt >= 8)
        {
            u8SpdCnt = 0;

            /* 计算id、iq目标值 */
            motor.id_ref = 0;
            motor.iq_ref = pid_regulation(motor.speed_ref, motor.speed_mech, &(motor.pid_speed));
            // motor.speed_ref_ramp = speed_ref_ramp(motor.speed_ref, motor.speed_mech);
            // motor.iq_ref = pid_regulation(motor.speed_ref_ramp, motor.speed_mech, &(motor.pid_speed));
        }

        /* 电流环16KHz */
        clarke_amplitude(&motor.ialpha, &motor.ibeta, motor.ia, motor.ib);
        park(&motor.id, &motor.iq, motor.ialpha, motor.ibeta, phase_sincos);
        UTILS_LP_FAST(motor.id_lpf, motor.id, 0.1f);
        UTILS_LP_FAST(motor.iq_lpf, motor.iq, 0.1f);
        motor.ud = pid_regulation(motor.id_ref, motor.id, &motor.pid_flux);
        motor.uq = pid_regulation(motor.iq_ref, motor.iq, &motor.pid_torque);
        over_modulation(&motor);
        rev_park(&motor.ualpha, &motor.ubeta, motor.ud, motor.uq, phase_sincos);
        break;
    case MC_MODE_POSITION:
        /* 位置环200Hz */
        u8PosCnt++;
        if(u8PosCnt >= 80)
        {
            u8PosCnt = 0;

            /* 关节位置控制 */
            motor.position_ref_plan = position_ref_plan_step(motor.position_ref, motor.angle_joint);
            f64PosError = position_error_normalize(motor.position_ref_plan, motor.angle_joint);
            motor.speed_ref = pid_regulation(f64PosError, 0, &motor.pid_position) * REDUCTION_RATIO;

            /* 转子位置控制 */
            // f64PosError = position_error_normalize(motor.position_ref, motor.angle_mech);
            // motor.speed_ref = pid_regulation(f64PosError, 0, &motor.pid_position);
        }

        /* 速度环2KHz */
        u8SpdCnt++;
        if(u8SpdCnt >= 8)
        {
            u8SpdCnt = 0;

            /* 计算id、iq参考值 */
            motor.id_ref = 0;
            motor.iq_ref = pid_regulation(motor.speed_ref, motor.speed_mech, &(motor.pid_speed));
        }

        /* 电流环16KHz */
        clarke_amplitude(&motor.ialpha, &motor.ibeta, motor.ia, motor.ib);
        park(&motor.id, &motor.iq, motor.ialpha, motor.ibeta, phase_sincos);
        UTILS_LP_FAST(motor.id_lpf, motor.id, 0.1f);
        UTILS_LP_FAST(motor.iq_lpf, motor.iq, 0.1f);
        motor.ud = pid_regulation(motor.id_ref, motor.id, &motor.pid_flux);
        motor.uq = pid_regulation(motor.iq_ref, motor.iq, &motor.pid_torque);
        over_modulation(&motor);
        rev_park(&motor.ualpha, &motor.ubeta, motor.ud, motor.uq, phase_sincos);
        break;
    default:
        break;
    }

    /* SVPWM调制 */
    motor_pwm_set(&motor);
}
