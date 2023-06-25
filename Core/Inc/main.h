/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SONAR2_TRIG_Pin GPIO_PIN_6
#define SONAR2_TRIG_GPIO_Port GPIOA
#define SONAR2_ECHO_Pin GPIO_PIN_7
#define SONAR2_ECHO_GPIO_Port GPIOA
#define SONAR3_TRIG_Pin GPIO_PIN_10
#define SONAR3_TRIG_GPIO_Port GPIOB
#define SONAR1_ECHO_Pin GPIO_PIN_7
#define SONAR1_ECHO_GPIO_Port GPIOC
#define sound_Pin GPIO_PIN_8
#define sound_GPIO_Port GPIOA
#define SONAR1_TRIG_Pin GPIO_PIN_9
#define SONAR1_TRIG_GPIO_Port GPIOA
#define SONAR3_ECHO_Pin GPIO_PIN_6
#define SONAR3_ECHO_GPIO_Port GPIOB
#define Left_Vibration_Pin GPIO_PIN_8
#define Left_Vibration_GPIO_Port GPIOB
#define Right_Vibration_Pin GPIO_PIN_9
#define Right_Vibration_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
