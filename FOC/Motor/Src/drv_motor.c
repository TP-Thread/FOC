/**
 * @file drv_motor.c
 * @author dufuqi (dufuqi@roborock.com)
 * @brief 电机驱动相关函数实现
 * @version 0.1
 * @date 2026-04-01
 * 
 * @copyright Copyright (c) 2026 roborock
 * 
 */

#include "drv_motor.h"
#include "main.h"
#include "systick.h"

/**
 * @brief 外设时钟配置
 * 
 */
static void motor_rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    
    rcu_periph_clock_enable(RCU_AF);
    
    rcu_periph_clock_enable(RCU_TIMER0);

    rcu_periph_clock_enable(RCU_SPI2);

    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
}

/**
 * @brief 配置GPIO引脚
 * 
 */
static void motor_gpio_config(void)
{
    /* configure nSLEEP pin */
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    /* configure nFAULT pin */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* configure timer0 pin */
    gpio_init(PWM_UPPER_A_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_UPPER_A_PIN);
    gpio_init(PWM_UPPER_B_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_UPPER_B_PIN);
    gpio_init(PWM_UPPER_C_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_UPPER_C_PIN);
    gpio_init(PWM_DOWN_A_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_DOWN_A_PIN);
    gpio_init(PWM_DOWN_B_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_DOWN_B_PIN);
    gpio_init(PWM_DOWN_C_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_DOWN_C_PIN);
    
    /* configure ADC pin */
    gpio_init(CURRENT_A_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, CURRENT_A_PIN);
    gpio_init(CURRENT_B_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, CURRENT_B_PIN);

    /* configure Encoder pin: SCK/PB3, MISO/PB4, MOSI/PB5, CS1/PA15, CS2/PB6 */
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE); /* 调试引脚切换 */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
    // gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
}

/**
 * @brief 配置TIMER0为FOC PWM模式，CH0/CH1/CH2输出PWM，CH3作为采样触发信号输出
 * 
 */
static void motor_timer0_config(void)
{
    timer_parameter_struct       timer_initpara;
    timer_oc_parameter_struct    timer_ocintpara;
    // timer_break_parameter_struct timer_breakpara;
    timer_deinit(TIMER0);

    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection  = TIMER_COUNTER_DOWN;
    timer_initpara.period            = PWM_COUNT;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0; 
    timer_init(TIMER0, &timer_initpara);
    timer_auto_reload_shadow_enable(TIMER0);
    
    /* TIMER0 output disable -- CH0/CH1/CH2 and CH0N/CH1N/CH2N */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.outputstate  = TIMER_CCX_DISABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);

    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);

    /* channel 3 configuration in OC */ 
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, PWM_COUNT - 10); /* 采样时刻 */
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM1);    /* PWM1上升沿触发采样*/
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_ENABLE);

    /* TIMER break time */
    // timer_break_struct_para_init(&timer_breakpara);
    // timer_breakpara.runoffstate        = TIMER_ROS_STATE_ENABLE;
    // timer_breakpara.ideloffstate       = TIMER_IOS_STATE_ENABLE;
    // timer_breakpara.protectmode        = TIMER_CCHP_PROT_OFF; 
    // timer_breakpara.deadtime           = 0;                        /* 驱动器自带死区，MCU不需要设置 */
    // timer_breakpara.breakstate         = TIMER_BREAK_DISABLE;      /* 启用刹车功能 */
    // timer_breakpara.breakpolarity      = TIMER_BREAK_POLARITY_LOW; /* fault引脚低电平有效（外部上拉） */
    // timer_breakpara.outputautostate    = TIMER_OUTAUTO_ENABLE;     /* fault解除后自动恢复PWM输出 */
    // timer_break_config(TIMER0, &timer_breakpara);

    timer_master_output_trigger_source_select(TIMER0, TIMER_TRI_OUT_SRC_UPDATE);
    timer_master_slave_mode_config(TIMER0, TIMER_MASTER_SLAVE_MODE_ENABLE);

    timer_primary_output_config(TIMER0, ENABLE);
}

/**
 * @brief 编码器外设初始化
 * 
 */
static void motor_spi2_config(void)
{
    spi_parameter_struct spi_init_struct;

    /* SPI2 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_8;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI2, &spi_init_struct);
    spi_enable(SPI2);

    /* Initialize CS pins to high */
    gpio_bit_set(GPIOA, GPIO_PIN_15);  /* CS1 */
    gpio_bit_set(GPIOB, GPIO_PIN_6);   /* CS2 */
}

/**
 * @brief 采样电流零位校准，计算ADC偏移值
 * 
 * @param motor 
 */
static void current_calibrate(motor_t* motor)
{
    uint8_t i;
    uint16_t current_sum_a = 0,current_sum_b = 0;
    
    /* down legs: closed, upper legs: open */
    gpio_bit_reset(PWM_DOWN_A_PORT, PWM_DOWN_A_PIN);
    gpio_bit_reset(PWM_DOWN_B_PORT, PWM_DOWN_B_PIN);
    gpio_bit_reset(PWM_DOWN_C_PORT, PWM_DOWN_C_PIN);
    gpio_bit_reset(PWM_UPPER_A_PORT, PWM_UPPER_A_PIN);
    gpio_bit_reset(PWM_UPPER_B_PORT, PWM_UPPER_B_PIN);
    gpio_bit_reset(PWM_UPPER_C_PORT, PWM_UPPER_C_PIN);
    gpio_init(PWM_DOWN_A_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, PWM_DOWN_A_PIN);
    gpio_init(PWM_DOWN_B_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, PWM_DOWN_B_PIN);
    gpio_init(PWM_DOWN_C_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, PWM_DOWN_C_PIN);
    gpio_init(PWM_UPPER_A_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, PWM_UPPER_A_PIN);
    gpio_init(PWM_UPPER_B_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, PWM_UPPER_B_PIN);
    gpio_init(PWM_UPPER_C_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, PWM_UPPER_C_PIN);

    /* use regular group to sample the current offset */
    /* the current offset of phase A */
    adc_regular_channel_config(ADC0, 0, CURRENT_A_CHANNEL, ADC_SAMPLETIME_239POINT5);
    for(i=0; i<26; i++){
        adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
        while(RESET == adc_flag_get(ADC0, ADC_FLAG_EOC));
        if(i >= 10){
            current_sum_a += adc_regular_data_read(ADC0);
        }
    }
    motor->ia_offset = current_sum_a/16;

    /* the current offset of phase B */
    adc_regular_channel_config(ADC0, 0, CURRENT_B_CHANNEL, ADC_SAMPLETIME_239POINT5);
    for(i=0; i<26; i++){
        adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
        while(RESET == adc_flag_get(ADC0, ADC_FLAG_EOC));
        if(i >= 10){
            current_sum_b += adc_regular_data_read(ADC0);
        }
    }
    motor->ib_offset = current_sum_b/16;
    
    /* restore to default configuration */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, DISABLE);

    adc_inserted_channel_config(ADC0, 0, CURRENT_A_CHANNEL, ADC_SAMPLETIME_1POINT5);
    adc_inserted_channel_config(ADC1, 0, CURRENT_B_CHANNEL, ADC_SAMPLETIME_1POINT5);

    /* release driver */
    gpio_init(PWM_DOWN_A_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_DOWN_A_PIN);
    gpio_init(PWM_DOWN_B_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_DOWN_B_PIN);
    gpio_init(PWM_DOWN_C_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_DOWN_C_PIN);
    gpio_init(PWM_UPPER_A_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_UPPER_A_PIN);
    gpio_init(PWM_UPPER_B_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_UPPER_B_PIN);
    gpio_init(PWM_UPPER_C_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_UPPER_C_PIN);
}

/**
 * @brief 配置2shunt电流采样ADC
 * 
 */
static void motor_adc_config(motor_t* motor)
{
    adc_deinit(ADC0);
    adc_deinit(ADC1);
    
    /* 配置双ADC并行模式 */
    adc_mode_config(ADC_DAUL_INSERTED_PARALLEL);
    /* ADC scan mode function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);

    /* configure ADC data alignment */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);

    /* configure ADC regular channel trigger */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); /* 规则通道软件触发 */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

    /* configure ADC regular channel length */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);
    adc_regular_channel_config(ADC0, 0, CURRENT_A_CHANNEL, ADC_SAMPLETIME_239POINT5); /* 规则通道用于电流零位校准，校准完再设置为注入通道 */

    /* configure ADC inserted channel trigger */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_CH3); /* ADC0的插入通道由TIMER0的CH3触发 */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);
    /* configure ADC inserted channel trigger */
    adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE); /* ADC1从属于ADC0，无需独立触发 */
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 1);
    /* configure ADC inserted channel */
    adc_inserted_channel_config(ADC0, 0, CURRENT_A_CHANNEL, ADC_SAMPLETIME_1POINT5);
    adc_inserted_channel_config(ADC1, 0, CURRENT_B_CHANNEL, ADC_SAMPLETIME_1POINT5);

    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADC0);
    adc_enable(ADC1);

    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    adc_calibration_enable(ADC1);

    /* 校准需要高精度，长采样时间，所以先配置成规则通道，校准完恢复注入通道 */
    current_calibrate(motor);
    
    nvic_irq_enable(ADC0_1_IRQn, 0, 0);
}

/**
 * @brief 电机PWM输出和电流采样外设初始化
 * 
 */
void motor_init(motor_t* motor)
{
    motor_rcu_config();
    motor_gpio_config();
    motor_timer0_config();
    motor_spi2_config();
    motor_adc_config(motor);
}

/********************************************************************************
 * 电机控制
 *******************************************************************************/

/**
 * @brief 电机启动，TIMER0 CH0/CH1/CH2输出PWM，CH3输出采样触发信号
 * 
 */
void motor_start(void)
{
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
    timer_event_software_generate(TIMER0, TIMER_EVENT_SRC_UPG);
    timer_enable(TIMER0);

    /* 使能ADC注入组转换完成中断 */
    adc_interrupt_flag_clear(ADC0, ADC_INT_EOIC);
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
}

/**
 * @brief 电机停止，TIMER0 CH0/CH1/CH2和CH0N/CH1N/CH2N输出禁止，TIMER0停止，ADC采样触发禁止，ADC中断禁止
 * 
 */
void motor_stop(void)
{
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);
    /* disable TIMER0 */
    timer_disable(TIMER0);

    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, DISABLE);
    adc_interrupt_disable(ADC0, ADC_INT_EOIC);
}

/*!
    \brief      the low-side mosfets output the fixed duty pwm
*/
static void motor_patial_brake(uint8_t duty)
{
    uint16_t pwm_set = 0;
    uint16_t pwm_top = 0;
    pwm_top = TIMER_CAR(TIMER0);

    pwm_set = pwm_top - pwm_top * duty / 100;

    timer_update_event_disable(TIMER0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pwm_set);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pwm_set);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, pwm_set);
    timer_update_event_enable(TIMER0);

    /* 下桥臂PWM斩波，电机绕组通过下桥臂和续流二极管形成短路回路，电机反电动势产生制动电流，消耗电机动能 */
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
}

/**
 * @brief 电机制动，分两阶段：第一阶段20%占空比初步降速，第二阶段0%占空比三相短路快速停机
 * 
 */
void motor_brake(void)
{
    /* 停止FOC控制 */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, DISABLE);
    adc_interrupt_disable(ADC0, ADC_INT_EOIC);

    /* 第一阶段制动 - 20%占空比，初步降速，避免冲击 */
    motor_patial_brake(20);

    delay_1ms(30);

    /* 第二阶段制动 - 0%占空比，三相短路，快速停机 */
    motor_patial_brake(0);
}

/********************************************************************************
 * 角度/速度计算
 *******************************************************************************/
/**
 * @brief SPI通信基础函数，发送和接收一个字节数据
 * 
 */
static uint8_t encoder_spi_rw(uint32_t spi_periph, uint8_t tx_data)
{
    uint8_t rx_data = 0;
    
    /* 等待发送缓冲区为空 */
    while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE))
        ;
    
    /* 发送数据 */
    spi_i2s_data_transmit(spi_periph, tx_data);
    
    /* 等待接收缓冲区非空 */
    while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE))
        ;
    
    /* 接收数据 */
    rx_data = spi_i2s_data_receive(spi_periph);
    
    return rx_data;
}

/**
 * @brief 读取MT6816原始角度值（14bits）
 * 
 */
void motor_raw_angle_get(motor_t *motor)
{
    uint8_t data_high = 0;
    uint8_t data_low = 0;
    
    /* 第一次SPI通讯：读取高字节（包含14位角度的高8位）*/
    gpio_bit_reset(GPIOA, GPIO_PIN_15);
    encoder_spi_rw(SPI2, 0x83);
    data_high = encoder_spi_rw(SPI2, 0xFF);
    gpio_bit_set(GPIOA, GPIO_PIN_15);
    
    /* 延迟以确保设备准备好 */
    volatile uint32_t delay = 10;
    while(delay--);
    
    /* 第二次SPI通讯：读取低字节（包含14位角度的低6位）*/
    gpio_bit_reset(GPIOA, GPIO_PIN_15);
    encoder_spi_rw(SPI2, 0x84);
    data_low = encoder_spi_rw(SPI2, 0xFF);
    gpio_bit_set(GPIOA, GPIO_PIN_15);
    
    /* 合并数据：高8位 + 低6位 = 14位角度值 */
    motor->angle_raw = ((uint16_t)data_high << 6) | ((uint16_t)data_low >> 2);
}

/**
 * @brief 获取转子电角度
 */
void motor_angle_speed_get(motor_t *motor)
{
    static uint8_t angle_track_init = 0;
    static float mech_angle_pre = 0.0f;
    static float mech_angle_total = 0.0f;

    /* 获取绝对值编码器角度 */
    motor_raw_angle_get(motor); 

    /* 机械角度 */
    motor->angle_mech = (float)motor->angle_raw * ENCODER_MECH_STEP;

    /* 解包电机机械角到多圈角，再折算为减速后关节角 */
    if (0 == angle_track_init)
    {
        mech_angle_pre = motor->angle_mech;
        mech_angle_total = motor->angle_mech;
        angle_track_init = 1;
    }
    else
    {
        float d_mech = motor->angle_mech - mech_angle_pre;

        if (d_mech > MOTOR_PI)
        {
            d_mech -= MOTOR_PI_2;
        }
        else if (d_mech < -MOTOR_PI)
        {
            d_mech += MOTOR_PI_2;
        }

        mech_angle_total += d_mech;
        mech_angle_pre = motor->angle_mech;
    }

    motor->angle_joint = mech_angle_total / REDUCTION_RATIO;
    while (motor->angle_joint >= MOTOR_PI_2)
    {
        motor->angle_joint -= MOTOR_PI_2;
    }
    while (motor->angle_joint < 0.0f)
    {
        motor->angle_joint += MOTOR_PI_2;
    }

    /* 应用软件零位偏移，使输出轴角度以用户定义零点表示 */
    motor->angle_joint -= motor->joint_offset;
    while (motor->angle_joint >= MOTOR_PI_2)
    {
        motor->angle_joint -= MOTOR_PI_2;
    }
    while (motor->angle_joint < 0.0f)
    {
        motor->angle_joint += MOTOR_PI_2;
    }

    motor->speed_joint = motor->speed_mech / REDUCTION_RATIO;

    if (CUSTOM_MODE == motor->angle_mode) /* 角度开环模式 */
    {
        static float f64AngleSet = 0;
        int16_t num;

        f64AngleSet += 0.0005f;
        num = (int16_t)(f64AngleSet / MOTOR_PI_2);

        if (f64AngleSet > 0)
        {
            motor->angle_elec = (f64AngleSet - num * MOTOR_PI_2);
        }
        else
        {
            motor->angle_elec = f64AngleSet - (num - 1) * MOTOR_PI_2;
        }
    }
    else if (ENCODER_MODE == motor->angle_mode) /* 编码器模式 */
    {
        int16_t num;
        float speed_temp;

        /* 电角度，规范化到[0, 2π) */
        motor->angle_elec = (motor->angle_raw - motor->angle_offset) > 0 ? ((motor->angle_raw - motor->angle_offset) * ENCODER_ELEC_STEP) : ((motor->angle_raw - motor->angle_offset + ENCODER_COUNT_NUM) * ENCODER_ELEC_STEP);
        num = (int16_t)((motor->angle_elec) / MOTOR_PI_2);
        motor->angle_elec = (motor->angle_elec - num * MOTOR_PI_2);

        /* 计算角速度，考虑到实际转速上限，50us电角度不会超过π */
        if ((motor->angle_elec - motor->angle_elec_pre) < -MOTOR_PI)
        {
            speed_temp = motor->angle_elec - motor->angle_elec_pre + MOTOR_PI_2;
        }
        else if ((motor->angle_elec - motor->angle_elec_pre) > MOTOR_PI)
        {
            speed_temp = motor->angle_elec - motor->angle_elec_pre - MOTOR_PI_2;
        }
        else
        {
            speed_temp = motor->angle_elec - motor->angle_elec_pre;
        }

        motor->angle_elec_pre = motor->angle_elec;

        /* 低通滤波器更新电角速度rad/s */
        UTILS_LP_FAST(motor->speed_elec, speed_temp * (float)PWM_FREQUENCE, 0.02f);

        /* 将电角速度转换为机械转速rpm */
        motor->speed_mech = motor->speed_elec * ENCODER_SPEED_FACTOR;
        motor->speed_joint = motor->speed_mech / REDUCTION_RATIO;

    }
}

void motor_joint_zero_set(motor_t *motor)
{
    motor_angle_speed_get(motor);
    motor->joint_offset = motor->angle_joint;
}

/********************************************************************************
 * 相电流计算
 *******************************************************************************/
/**
 * @brief 获取三相电流值
 * 
 */
void motor_current_get(motor_t* motor)
{
    int16_t current_a = 0, current_b = 0 ,current_c = 0;
    current_a = motor->ia_offset - adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
    current_b = motor->ib_offset - adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);
    // current_a = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0) - motor->ia_offset;
    // current_b = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0) - motor->ib_offset;
    current_c = -(current_a + current_b);
    
    motor->ia = ((float)current_a * CURRENT_FACTOR);
    motor->ib = ((float)current_b * CURRENT_FACTOR);
    motor->ic = ((float)current_c * CURRENT_FACTOR);
}

/********************************************************************************
 * SVPWM调制
 *******************************************************************************/
/**
 * @brief 7段式SVPWM调制
 * 
 */
static void csvpwm(motor_t *motor)
{
    float alpha, beta;

    /* voltage normalization */
    alpha = (float)motor->ualpha / motor->us_base;
    beta = (float)motor->ubeta / motor->us_base;

    /* 扇区判断 */
    if (beta >= 0.0f)
    {
        if (alpha >= 0.0f)
        {
            if (ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->pwm.sector = TWO;
            else
                motor->pwm.sector = ONE;
        }
        else
        {
            if (-ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->pwm.sector = THREE;
            else
                motor->pwm.sector = TWO;
        }
    }
    else
    {
        if (alpha >= 0.0f)
        {
            if (-ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->pwm.sector = FIVE;
            else
                motor->pwm.sector = SIX;
        }
        else
        {
            if (ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->pwm.sector = FOUR;
            else
                motor->pwm.sector = FIVE;
        }
    }

    /* 7段式SVPWM时间分配 */
    switch (motor->pwm.sector)
    {
    case ONE:
    {
        uint32_t t4 = (uint32_t)((alpha - ONE_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);
        uint32_t t6 = (uint32_t)((TWO_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);

        motor->pwm.tC = (motor->pwm.pwm_top - t4 - t6) >> 1;
        motor->pwm.tB = motor->pwm.tC + t6;
        motor->pwm.tA = motor->pwm.tB + t4;
        break;
    }
    case TWO:
    {
        uint32_t t6 = (uint32_t)((alpha + ONE_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);
        uint32_t t2 = (uint32_t)((-alpha + ONE_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);

        motor->pwm.tC = (motor->pwm.pwm_top - t6 - t2) >> 1;
        motor->pwm.tA = motor->pwm.tC + t6;
        motor->pwm.tB = motor->pwm.tA + t2;
        break;
    }
    case THREE:
    {
        uint32_t t2 = (uint32_t)((TWO_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);
        uint32_t t3 = (uint32_t)((-alpha - ONE_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);

        motor->pwm.tA = (motor->pwm.pwm_top - t2 - t3) >> 1;
        motor->pwm.tC = motor->pwm.tA + t3;
        motor->pwm.tB = motor->pwm.tC + t2;
        break;
    }
    case FOUR:
    {
        uint32_t t3 = (uint32_t)((-alpha + ONE_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);
        uint32_t t1 = (uint32_t)((-TWO_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);

        motor->pwm.tA = (motor->pwm.pwm_top - t3 - t1) >> 1;
        motor->pwm.tB = motor->pwm.tA + t3;
        motor->pwm.tC = motor->pwm.tB + t1;
        break;
    }
    case FIVE:
    {
        uint32_t t1 = (uint32_t)((-alpha - ONE_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);
        uint32_t t5 = (uint32_t)((alpha - ONE_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);

        motor->pwm.tB = (motor->pwm.pwm_top - t5 - t1) >> 1;
        motor->pwm.tA = motor->pwm.tB + t5;
        motor->pwm.tC = motor->pwm.tA + t1;
        break;
    }
    case SIX:
    {
        uint32_t t5 = (uint32_t)((-TWO_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);
        uint32_t t4 = (uint32_t)((alpha + ONE_DIVIDE_SQRT3 * beta) * motor->pwm.pwm_top);

        motor->pwm.tB = (motor->pwm.pwm_top - t5 - t4) >> 1;
        motor->pwm.tC = motor->pwm.tB + t5;
        motor->pwm.tA = motor->pwm.tC + t4;
        break;
    }
    default:
        break;
    }
}

/**
 * @brief 更新PWM占空比
 * 
 */
void motor_pwm_set(motor_t* motor)
{
    csvpwm(motor);
    
    timer_update_event_disable(TIMER0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, motor->pwm.tA);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, motor->pwm.tB);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, motor->pwm.tC);
    timer_update_event_enable(TIMER0);
}
