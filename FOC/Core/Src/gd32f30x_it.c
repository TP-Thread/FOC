/*!
    \file    gd32f30x_it.c
    \brief   FOC interrupt service routines

    \version 2025-06-30, V1.0.0, GDFOC2.0 for GD32F30x
*/

#include "gd32f30x_it.h"
#include "systick.h"
#include "main.h"
#include "motor_ctrl.h"
#include "motor_protect.h"

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    /* delay decrement */
    delay_decrement();

    /* 电机过压/过流/堵转保护 */
    motor_protect_check();
}

/*!
    \brief      this function handles FOC control
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ADC0_1_IRQHandler(void)
{
    /* clear the interrupt flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_EOIC);

    /* FOC control */
    motor_ctrl();
}
