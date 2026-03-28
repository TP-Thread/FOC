/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_15
#define LED_R_GPIO_Port GPIOC
#define SD4_Pin GPIO_PIN_0
#define SD4_GPIO_Port GPIOA
#define SD2_Pin GPIO_PIN_1
#define SD2_GPIO_Port GPIOA
#define SD3_Pin GPIO_PIN_2
#define SD3_GPIO_Port GPIOA
#define IC_Pin GPIO_PIN_4
#define IC_GPIO_Port GPIOA
#define IA_Pin GPIO_PIN_6
#define IA_GPIO_Port GPIOA
#define VBUS_Pin GPIO_PIN_5
#define VBUS_GPIO_Port GPIOC
#define SD1_Pin GPIO_PIN_0
#define SD1_GPIO_Port GPIOB
#define GEAR_Pin GPIO_PIN_2
#define GEAR_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_12
#define KEY2_GPIO_Port GPIOB
#define TEMP_Pin GPIO_PIN_15
#define TEMP_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_9
#define KEY1_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_11
#define LCD_RST_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_2
#define LCD_RS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define led_r_on()     HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)    
#define led_r_off()    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)     
#define led_r_toggle() HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin)	
#define led_g_on()     HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET)    
#define led_g_off()    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET)     
#define led_g_toggle() HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin)	
#define led_b_on()     HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET)    
#define led_b_off()    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET)     
#define led_b_toggle() HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
