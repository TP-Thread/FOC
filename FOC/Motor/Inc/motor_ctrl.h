/**
 * @file motor_ctrl.h
 * @author dufuqi (dufuqi@roborock.com)
 * @brief 电机控制相关
 * @version 0.1
 * @date 2026-04-01
 * 
 * @copyright Copyright (c) 2026 roborock
 * 
 */

#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "foc_math.h"

#define RR_MOTOR_60
// #define RR_MOTOR_35

#ifdef RR_MOTOR_60
    #define POLE_PAIRS                          (11U)
    #define REDUCTION_RATIO                     (33.284f)

    #define SPEED_REFERENCE_MAX                 (2900.f)
    #define SPEED_REFERENCE_MIN                 (-2900.f)
    #define SPEED_RAMP_STEP                     (10.0f)

    #define POSITION_TRAJ_STEP                  (0.1f)

    /* PID参数 */
    #define CURRENT_LOOP_KP                     (1.0f)
    #define CURRENT_LOOP_KI                     (0.001f)
    #define CURRENT_LOOP_SUM_MAX                (13.0f)
    #define CURRENT_LOOP_SUM_MIN                (-13.0f)
    #define CURRENT_LOOP_UOTPUT_MAX             (13.856f)
    #define CURRENT_LOOP_UOTPUT_MIN             (-13.856f)

    #define SPEED_LOOP_KP                       (0.03f)
    #define SPEED_LOOP_KI                       (0.0002f)
    #define SPEED_LOOP_SUM_MAX                  (6.0f)
    #define SPEED_LOOP_SUM_MIN                  (-6.0f)
    #define SPEED_LOOP_UOTPUT_MAX               (8.0f)
    #define SPEED_LOOP_UOTPUT_MIN               (-8.0f)

    #define POSITION_LOOP_KP                    (150.0f)
    #define POSITION_LOOP_SUM_MAX               (400.0f)
    #define POSITION_LOOP_SUM_MIN               (-400.0f)
    #define POSITION_LOOP_UOTPUT_MAX            (400.0f)
    #define POSITION_LOOP_UOTPUT_MIN            (-400.0f)
#elif defined RR_MOTOR_35
    #define POLE_PAIRS                          (7U)
    #define REDUCTION_RATIO                     (42.843f)

    #define SPEED_REFERENCE_MAX                 (5000.f)
    #define SPEED_REFERENCE_MIN                 (-5000.f)
    #define SPEED_RAMP_STEP                     (10.0f)

    #define POSITION_TRAJ_STEP                  (0.1f)

    /* PID参数 */
    #define CURRENT_LOOP_KP                     (1.0f)
    #define CURRENT_LOOP_KI                     (0.001f)
    #define CURRENT_LOOP_SUM_MAX                (13.0f)
    #define CURRENT_LOOP_SUM_MIN                (-13.0f)
    #define CURRENT_LOOP_UOTPUT_MAX             (13.856f)
    #define CURRENT_LOOP_UOTPUT_MIN             (-13.856f)

    #define SPEED_LOOP_KP                       (0.01f)
    #define SPEED_LOOP_KI                       (0.00001f)
    #define SPEED_LOOP_SUM_MAX                  (6.0f)
    #define SPEED_LOOP_SUM_MIN                  (-6.0f)
    #define SPEED_LOOP_UOTPUT_MAX               (8.0f)
    #define SPEED_LOOP_UOTPUT_MIN               (-8.0f)

    #define POSITION_LOOP_KP                    (300.0f)
    #define POSITION_LOOP_SUM_MAX               (600.0f)
    #define POSITION_LOOP_SUM_MIN               (-600.0f)
    #define POSITION_LOOP_UOTPUT_MAX            (800.0f)
    #define POSITION_LOOP_UOTPUT_MIN            (-800.0f)
#endif

/* PWM parameters */
#define SYSTEM_FREQUENCE                    (120000000U)                        /* system frequence, uint: Hz */
#define PWM_FREQUENCE                       (16000U)                            /* PWM frequence, uint:Hz */
#define PWM_COUNT                           (SYSTEM_FREQUENCE/PWM_FREQUENCE/2)  /* 3750 定时器重装载值 */

#define FMC_OFFSET_ANGLE_ADDR               ((uint32_t)0x0801E000U)             /* 电角度零位偏移值存储地址 */

typedef enum{
    MC_STATE_INIT = 0,                                                          /* init state */
    MC_STATE_IDLE,                                                              /* idle state */
    MC_STATE_RUNNING,                                                           /* running state */
    MC_STATE_BRAKE,                                                             /* brake state */
    MC_STATE_FAULT                                                              /* fault state */
} mc_state_e;

typedef enum{
    MC_MODE_OPEN = 0,                                                           /* 电流开环 */
    MC_MODE_CALIB,                                                              /* 编码器校准 */
    MC_MODE_CURRENT,                                                            /* 电流闭环 */
    MC_MODE_SPEED,                                                              /* 速度闭环 */
    MC_MODE_POSITION,                                                           /* 位置闭环 */
} mc_mode_e;

typedef enum{
    FAULT_NONE = 0,                                                             /* no faults */
    FAULT_VOLTAGE_ERROR,                                                        /* under voltage or over voltage fault */
    FAULT_OVER_CURRENT,                                                         /* over current fault */
    FAULT_MOTOR_LOCKED,                                                         /* blocking fault */
    FAULT_SPEED_ERROR,                                                          /* blocking fault */
    FAULT_PHASE_LOSS,                                                           /* phase loss fault */
    FAULT_ANGLE
} mc_fault_e;

typedef enum
{
    CUSTOM_MODE = 0,                                                            /* open loop mode */  
    ENCODER_MODE,
} mc_angle_mode_e;

typedef struct{
    mc_state_e state;
    mc_mode_e mode;
    mc_fault_e fault;
    mc_angle_mode_e angle_mode;                                                 /* 角度获取方式 */

    /* motor body parameter */
    uint8_t pole_pairs;
    float us_base;
    float us_out_max;

    /* angle and speed */
    uint16_t angle_raw;                                                         /* MT6816 raw angle data */
    uint16_t angle_offset;                                                      /* 电角度零位补偿角 */
    
    float angle_elec;
    float angle_elec_pre;
    float speed_elec;                                                           /* electric speed, rad/s */

    float angle_mech;                                                           /* mechanical angle, deg */
    float speed_mech;                                                           /* mechanical speed, r/min */

    float angle_joint;
    float joint_offset;                                                         /* 软件零位偏移，rad */
    float speed_joint;                                                          /* joint speed, r/min */
    
    /* physical quantities */
    uint16_t ia_offset;
    uint16_t ib_offset;
    uint16_t ic_offset;
    float ia;
    float ib;
    float ic;

    float ialpha;
    float ibeta;

    float id;
    float iq;
    float id_lpf;
    float iq_lpf;

    float ud;
    float uq;

    float ualpha;
    float ubeta;

    /* physical quantities reference */
    float id_ref;
    float iq_ref;
    float speed_ref;
    float speed_ref_ramp;
    float position_ref;
    float position_ref_plan;

    /* pid struct */
    pid_t pid_flux;
    pid_t pid_torque;
    pid_t pid_speed;
    pid_t pid_position;

    /* PWM struct */
    pwm_t pwm;
} motor_t;

extern motor_t motor;

void motor_ctrl(void);

#endif /* MOTOR_CTRL_H */
