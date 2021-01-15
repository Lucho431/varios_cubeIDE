/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define boton_verde_Pin GPIO_PIN_1
#define boton_verde_GPIO_Port GPIOA
#define boton_naranja_Pin GPIO_PIN_3
#define boton_naranja_GPIO_Port GPIOA
#define boton_rojo_Pin GPIO_PIN_5
#define boton_rojo_GPIO_Port GPIOA
#define boton_azul_Pin GPIO_PIN_7
#define boton_azul_GPIO_Port GPIOA
#define led_verde_Pin GPIO_PIN_12
#define led_verde_GPIO_Port GPIOD
#define led_naranja_Pin GPIO_PIN_13
#define led_naranja_GPIO_Port GPIOD
#define led_rojo_Pin GPIO_PIN_14
#define led_rojo_GPIO_Port GPIOD
#define led_azul_Pin GPIO_PIN_15
#define led_azul_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
