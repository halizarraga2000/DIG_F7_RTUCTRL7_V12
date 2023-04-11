/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_Rojo_Pin GPIO_PIN_3
#define Led_Rojo_GPIO_Port GPIOE
#define SW_01_Pin GPIO_PIN_4
#define SW_01_GPIO_Port GPIOE
#define SW_02_Pin GPIO_PIN_5
#define SW_02_GPIO_Port GPIOE
#define SW_03_Pin GPIO_PIN_6
#define SW_03_GPIO_Port GPIOE
#define SW_04_Pin GPIO_PIN_13
#define SW_04_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_2
#define Buzzer_GPIO_Port GPIOC
#define DI_AN_Pin GPIO_PIN_4
#define DI_AN_GPIO_Port GPIOA
#define SPIV2_ENT_Pin GPIO_PIN_6
#define SPIV2_ENT_GPIO_Port GPIOA
#define SERIAL_OUT_Pin GPIO_PIN_0
#define SERIAL_OUT_GPIO_Port GPIOB
#define LATCH_Pin GPIO_PIN_1
#define LATCH_GPIO_Port GPIOB
#define UART7_RXD_Pin GPIO_PIN_10
#define UART7_RXD_GPIO_Port GPIOE
#define SPIV2_P_S_Pin GPIO_PIN_11
#define SPIV2_P_S_GPIO_Port GPIOE
#define SPIV2_CLK_Pin GPIO_PIN_12
#define SPIV2_CLK_GPIO_Port GPIOE
#define SPIV2_CODE_Pin GPIO_PIN_13
#define SPIV2_CODE_GPIO_Port GPIOE
#define DO_AN_Pin GPIO_PIN_14
#define DO_AN_GPIO_Port GPIOE
#define ENABLE_Pin GPIO_PIN_15
#define ENABLE_GPIO_Port GPIOE
#define SPIV2_V5LED_Pin GPIO_PIN_10
#define SPIV2_V5LED_GPIO_Port GPIOB
#define USB_PWR_ON_Pin GPIO_PIN_10
#define USB_PWR_ON_GPIO_Port GPIOD
#define USART3_DE_Pin GPIO_PIN_14
#define USART3_DE_GPIO_Port GPIOD
#define DS18B20_Pin GPIO_PIN_15
#define DS18B20_GPIO_Port GPIOD
#define EA1_Pin GPIO_PIN_9
#define EA1_GPIO_Port GPIOC
#define EA2_Pin GPIO_PIN_8
#define EA2_GPIO_Port GPIOA
#define EA3_Pin GPIO_PIN_10
#define EA3_GPIO_Port GPIOA
#define UART4_RXD_Pin GPIO_PIN_0
#define UART4_RXD_GPIO_Port GPIOD
#define USART2_RXD_Pin GPIO_PIN_7
#define USART2_RXD_GPIO_Port GPIOD
#define UART8_RXD_Pin GPIO_PIN_4
#define UART8_RXD_GPIO_Port GPIOB
#define Led_Azul_Pin GPIO_PIN_5
#define Led_Azul_GPIO_Port GPIOB
#define Led_Amarillo_Pin GPIO_PIN_7
#define Led_Amarillo_GPIO_Port GPIOB
#define Led_Verde_Pin GPIO_PIN_8
#define Led_Verde_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
