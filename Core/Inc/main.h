/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
enum ErrorCodes {
	I2CErrorWrite,
	I2CErrorRead,
	BQNotPresentOrNotCorrectPart
};
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RGB_R_Pin GPIO_PIN_0
#define RGB_R_GPIO_Port GPIOA
#define RGB_G_Pin GPIO_PIN_1
#define RGB_G_GPIO_Port GPIOA
#define RGB_B_Pin GPIO_PIN_2
#define RGB_B_GPIO_Port GPIOA
#define Power_Button_Pin GPIO_PIN_7
#define Power_Button_GPIO_Port GPIOC
#define Power_Button_EXTI_IRQn EXTI9_5_IRQn
#define MAX_ALRT_Pin GPIO_PIN_9
#define MAX_ALRT_GPIO_Port GPIOC
#define MAX_ALRT_EXTI_IRQn EXTI9_5_IRQn
#define BQ_INT_Pin GPIO_PIN_8
#define BQ_INT_GPIO_Port GPIOA
#define BQ_INT_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
