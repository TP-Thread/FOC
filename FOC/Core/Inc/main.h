/*!
    \file    main.h
    \brief   the header file of main 

    \version 2025-06-30, V1.0.0, GDFOC2.0 for GD32F30x
*/

#ifndef MAIN_H
#define MAIN_H

#include "gd32f30x.h"

/* current sampling parameter */
/* GPIO for sampling A-phase current */
#define CURRENT_A_PIN                       GPIO_PIN_3                      /* GPIO pin */
#define CURRENT_A_PORT                      GPIOA                           /* GPIO port */
#define CURRENT_A_CHANNEL                   ADC_CHANNEL_3                   /* ADC channel corresponding to GPIO pin */
/* GPIO for sampling B-phase current */
#define CURRENT_B_PIN                       GPIO_PIN_4                      /* GPIO pin */
#define CURRENT_B_PORT                      GPIOA                           /* GPIO port */
#define CURRENT_B_CHANNEL                   ADC_CHANNEL_4                   /* ADC channel corresponding to GPIO pin */

/* PWM wave generation */
/* GPIO for A-phase PWM */
#define PWM_UPPER_A_PORT                    GPIOA                           /* GPIO port */
#define PWM_UPPER_A_PIN                     GPIO_PIN_8                      /* GPIO pin */
#define PWM_DOWN_A_PORT                     GPIOB                           /* GPIO port */
#define PWM_DOWN_A_PIN                      GPIO_PIN_13                     /* GPIO pin */
/* GPIO for B-phase PWM */
#define PWM_UPPER_B_PORT                    GPIOA                           /* GPIO port */
#define PWM_UPPER_B_PIN                     GPIO_PIN_9                      /* GPIO pin */
#define PWM_DOWN_B_PORT                     GPIOB                           /* GPIO port */
#define PWM_DOWN_B_PIN                      GPIO_PIN_14
/* GPIO for C-phase PWM */
#define PWM_UPPER_C_PORT                    GPIOA                           /* GPIO port */
#define PWM_UPPER_C_PIN                     GPIO_PIN_10                     /* GPIO pin */
#define PWM_DOWN_C_PORT                     GPIOB                           /* GPIO port */
#define PWM_DOWN_C_PIN                      GPIO_PIN_15                     /* GPIO pin */

/* voltage sampling parameter */
/* GPIO for sampling bus voltage */
#define BUS_VOLTAGE_PIN                     GPIO_PIN_1                      /* GPIO pin */
#define BUS_VOLTAGE_PORT                    GPIOB                           /* GPIO port */
#define BUS_VOLTAGE_CHANNEL                 ADC_CHANNEL_9                   /* ADC channel corresponding to GPIO pin */
#define BUS_VOLTAGE_AMP_GAIN                (47.f / 407.f)                 /* the gain of amplifier in the current conditioning circuit */

#endif /* MAIN_H */
