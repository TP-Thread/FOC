/*!
    \file    motor_protect.c
    \brief   motor software protection program

    \version 2025-06-30, V1.0.0, GDFOC2.0 for GD32F30x
*/

#include "motor_protect.h"
#include "drv_motor.h"
#include "main.h"

/* ADC sampled bus voltage data */
static uint16_t adc_regular_buffer = 0;
/* the amplitude table of the ualpha and ubeta */
static float motor_lock_voltage[40] = {
    /* 0~900rpm */
    1.50f, 1.50f, 1.50f, 1.50f, 1.50f, 1.85f, 2.15f, 2.55f, 2.90f, 3.33f,
    /* 1000~1900rpm */
    3.69f, 4.05f, 4.35f, 4.65f, 5.02f, 5.35f, 5.70f, 6.07f, 6.42f, 6.75f,
    /* 2000~2900rpm */
    7.12f, 7.47f, 7.82f, 8.17f, 8.52f, 8.86f, 9.22f, 9.57f, 9.92f, 10.27f,
    /* 3000~3900rpm */
    10.62f, 10.96f, 11.31f, 11.66f, 12.01f, 12.35f, 12.70f, 13.02f, 13.02f, 13.02f};

motor_protect_struct motor_protect;

/*!
    \brief      configure peripheral clocks related to the motor protect
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_DMA0);
}

/*!
    \brief      configure the GPIO peripheral related to the motor protect
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_gpio_config(void)
{
    /* configure the GPIO peripheral */
    gpio_init(BUS_VOLTAGE_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, BUS_VOLTAGE_PIN);
}

/*!
    \brief      configure the ADC peripheral related to the motor protect
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_adc_config(void)
{
    /* configure ADC data alignment */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* configure ADC regular channel trigger */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

    /* configure ADC regular channel length */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);
    adc_regular_channel_config(ADC0, 0, BUS_VOLTAGE_CHANNEL, ADC_SAMPLETIME_7POINT5);

    /* enable DMA request */
    adc_dma_mode_enable(ADC0);
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
}

/*!
    \brief      configure the DMA peripheral related to the motor protect
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_dma_config(void)
{
    dma_parameter_struct dma_init_struct;

    /* DMA configuration */
    dma_deinit(DMA0, DMA_CH0);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)(&adc_regular_buffer);
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.number = 1;
    dma_init_struct.periph_addr = (uint32_t)(&ADC_RDATA(ADC0));
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init(DMA0, DMA_CH0, &dma_init_struct);
    dma_circulation_enable(DMA0, DMA_CH0);
    dma_memory_to_memory_disable(DMA0, DMA_CH0);

    /* enable the DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}

/*!
    \brief      initialize the motor protect peripherals
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_protect_peripheral_init(void)
{
    motor_protect_rcu_config();
    motor_protect_gpio_config();
    motor_protect_adc_config();
    motor_protect_dma_config();
}

/*!
    \brief      initialize the motor protect variables
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_protect_var_init(void)
{
    /* initialize motor protect struct variables */
    motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR] = 0;
    motor_protect.fault_iteration[FAULT_OVER_CURRENT] = 0;
    motor_protect.fault_iteration[FAULT_MOTOR_LOCKED] = 0;
    motor_protect.fault_iteration[FAULT_SPEED_ERROR] = 0;
    for (int i = 0; i < 3; i++)
    {
        motor_protect.low_current_count[i] = 0;
    }
    motor_protect.pwm_count = 0;
    motor_protect.start_time = (int16_t)(motor.speed_ref * START_TIME_SET_FACTOR);
    motor_protect.pre_speed_ref = motor.speed_ref;
}

/*!
    \brief      check whether the motor is undervoltage or overvoltage
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_voltage_check(void)
{
    /* 计算总线电压 */
    motor_protect.v_bus = adc_regular_buffer * BUS_VOLTAGE_CALC_FACTOR;

    /* 检查电压是否超出正常范围 */
    if ((motor_protect.v_bus < BUS_VOLTAGE_THRESHOLD_LOW) || (motor_protect.v_bus > BUS_VOLTAGE_THRESHOLD_HIGH))
    {
        motor_protect.fault_recovery[FAULT_VOLTAGE_ERROR] = 0;
        motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR]++;

        if (motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR] >= 250)
        {
            motor.fault = FAULT_VOLTAGE_ERROR;
        }
    }
    /* 检查电压是否恢复正常 */
    else if (motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR] > 0)
    {
        motor_protect.fault_recovery[FAULT_VOLTAGE_ERROR]++;

        if (motor_protect.fault_recovery[FAULT_VOLTAGE_ERROR] >= 1000)
        {
            motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR] = 0;

            if (motor.fault == FAULT_VOLTAGE_ERROR)
            {
                motor.fault = FAULT_NONE;
            }
        }
    }
}

/*!
    \brief      check whether the motor overcurrent
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_over_current_check(void)
{
    /* calculate the amplitude of the phase current */
    motor_protect.i_phase_amplit = sqrt_float(motor.ialpha * motor.ialpha + motor.ibeta * motor.ibeta);

    /* check whether the current exceeds the threshold */
    if (motor_protect.i_phase_amplit > PHASE_CURRENT_THRESHOLD_HIGH)
    {
        motor_protect.fault_recovery[FAULT_OVER_CURRENT] = 0;
        motor_protect.fault_iteration[FAULT_OVER_CURRENT]++;
        /* the fault count reaches the set value */
        if (motor_protect.fault_iteration[FAULT_OVER_CURRENT] >= 250)
        {
            motor.fault = FAULT_OVER_CURRENT;
        }
        /* check whether the current returns to normal */
    }
    else if (motor_protect.fault_iteration[FAULT_OVER_CURRENT] > 0)
    {
        motor_protect.fault_recovery[FAULT_OVER_CURRENT]++;
        /* the recovery count reaches the set value */
        if (motor_protect.fault_recovery[FAULT_OVER_CURRENT] >= 1000)
        {
            motor_protect.fault_iteration[FAULT_OVER_CURRENT] = 0;
            /* clear the fault */
            if (motor.fault == FAULT_OVER_CURRENT)
            {
                motor.fault = FAULT_NONE;
            }
        }
    }
}

/*!
    \brief      check whether the motor is locked
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_motor_locked_check(void)
{
    float voltage_threshold;
    /* calculate the amplitude of the ualpha and ubeta */
    motor_protect.v_phase_amplit = sqrt_float(motor.ualpha * motor.ualpha + motor.ubeta * motor.ubeta);

    if (motor_protect.start_time > 0)
    {
        /* 在电机启动阶段不开启故障保护 */
        motor_protect.start_time--;
    }
    else
    {
        /* 检查机械速度是否超出正常范围 */
        if ((motor.speed_mech > SPEED_THRESHOLD_HIGH) || (motor.speed_mech < SPEED_THRESHOLD_LOW))
        {
            motor_protect.fault_recovery[FAULT_SPEED_ERROR] = 0;
            motor_protect.fault_iteration[FAULT_SPEED_ERROR]++;

            if (motor_protect.fault_iteration[FAULT_SPEED_ERROR] >= 250)
            {
                motor.fault = FAULT_SPEED_ERROR;
            }
        }
        /* 检查机械速度是否恢复正常范围 */
        else if (motor_protect.fault_iteration[FAULT_SPEED_ERROR] > 0)
        {
            motor_protect.fault_recovery[FAULT_SPEED_ERROR]++;

            if (motor_protect.fault_recovery[FAULT_SPEED_ERROR] >= 1000)
            {
                motor_protect.fault_iteration[FAULT_SPEED_ERROR] = 0;
            }
        }

        /* calculate the voltage threshold */
        voltage_threshold = MOTOR_LOCKED_VOLT_RATIO * motor_lock_voltage[(uint8_t)(motor.speed_mech * 0.01f)];
        /* check whether the phase voltage amplitude exceeds the threshold */
        if (motor_protect.v_phase_amplit < voltage_threshold)
        {
            motor_protect.fault_recovery[FAULT_MOTOR_LOCKED] = 0;
            motor_protect.fault_iteration[FAULT_MOTOR_LOCKED]++;
            /* the fault count reaches the set value */
            if (motor_protect.fault_iteration[FAULT_MOTOR_LOCKED] >= 150)
            {
                motor.fault = FAULT_MOTOR_LOCKED;
            }
            /* check whether the phase voltage amplitude returns to normal */
        }
        else if (motor_protect.fault_iteration[FAULT_MOTOR_LOCKED] > 0)
        {
            motor_protect.fault_recovery[FAULT_MOTOR_LOCKED]++;
            /* the recovery count reaches the set value */
            if (motor_protect.fault_recovery[FAULT_MOTOR_LOCKED] >= 1000)
            {
                motor_protect.fault_iteration[FAULT_MOTOR_LOCKED] = 0;
            }
        }
    }

    /* check whether the speed reference changes */
    if (motor.speed_ref != motor_protect.pre_speed_ref)
    {
        /* setting the motor startup time */
        if (motor.speed_ref > motor.speed_mech)
        {
            motor_protect.start_time = (uint32_t)((motor.speed_ref - motor.speed_mech) * START_TIME_SET_FACTOR);
        }
        else
        {
            motor_protect.start_time = (uint32_t)((motor.speed_mech - motor.speed_ref) * START_TIME_SET_FACTOR);
        }
    }
    motor_protect.pre_speed_ref = motor.speed_ref;
}

/*!
    \brief      motor protect process
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_protect_check(void)
{
    /* 电压范围检测 */
    motor_protect_voltage_check();
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL); /* 触发常规通道采样（母线电压）*/

    /* check whether the motor overcurrent */
    // motor_protect_over_current_check();
    /* check whether the motor is locked */
    // motor_protect_motor_locked_check();
}

/*!
    \brief      check whether the motor phase loss
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_protect_phase_loss_check(void)
{
    if (motor_protect.pwm_count < PHASE_LOSS_COUNT_MAX)
    {
        motor_protect.pwm_count++;
        /* compare the phase current with the set threshold */
        if ((motor.ia < LOW_CURRENT_THRESHOLD) && (motor.ia > -LOW_CURRENT_THRESHOLD))
        {
            motor_protect.low_current_count[0]++;
        }
        if ((motor.ib < LOW_CURRENT_THRESHOLD) && (motor.ib > -LOW_CURRENT_THRESHOLD))
        {
            motor_protect.low_current_count[1]++;
        }
        if ((motor.ic < LOW_CURRENT_THRESHOLD) && (motor.ic > -LOW_CURRENT_THRESHOLD))
        {
            motor_protect.low_current_count[2]++;
        }
    }
    else
    {
        motor_protect.pwm_count = 0;
        /* the phase loss fault count reaches the set value */
        if ((motor_protect.low_current_count[0] > PHASE_LOSS_COUNT_THRESHOLD) ||
            (motor_protect.low_current_count[1] > PHASE_LOSS_COUNT_THRESHOLD) ||
            (motor_protect.low_current_count[2] > PHASE_LOSS_COUNT_THRESHOLD))
        {
            motor.fault = FAULT_PHASE_LOSS;
        }
        motor_protect.low_current_count[0] = 0;
        motor_protect.low_current_count[1] = 0;
        motor_protect.low_current_count[2] = 0;
    }
}
