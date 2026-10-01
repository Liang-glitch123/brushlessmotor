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
#define KEY3_Pin GPIO_PIN_13
#define KEY3_GPIO_Port GPIOC
#define Motor1_EMFU_Pin GPIO_PIN_0
#define Motor1_EMFU_GPIO_Port GPIOC
#define Motor1_EMFV_Pin GPIO_PIN_2
#define Motor1_EMFV_GPIO_Port GPIOC
#define Motor1_EMFW_Pin GPIO_PIN_3
#define Motor1_EMFW_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOA
#define Motor1_IU_Pin GPIO_PIN_3
#define Motor1_IU_GPIO_Port GPIOA
#define Motor1_IV_Pin GPIO_PIN_4
#define Motor1_IV_GPIO_Port GPIOA
#define Motor1_IW_Pin GPIO_PIN_6
#define Motor1_IW_GPIO_Port GPIOA
#define Motor1_VBUS_Pin GPIO_PIN_0
#define Motor1_VBUS_GPIO_Port GPIOB
#define Motor1_IBUS_Pin GPIO_PIN_1
#define Motor1_IBUS_GPIO_Port GPIOB
#define Motor1_UL_Pin GPIO_PIN_13
#define Motor1_UL_GPIO_Port GPIOB
#define Motor1_VL_Pin GPIO_PIN_14
#define Motor1_VL_GPIO_Port GPIOB
#define Motor1_WL_Pin GPIO_PIN_15
#define Motor1_WL_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOG
#define KEY4_Pin GPIO_PIN_3
#define KEY4_GPIO_Port GPIOG
#define KEY5_Pin GPIO_PIN_4
#define KEY5_GPIO_Port GPIOG
#define Motor1_EA_HU_Pin GPIO_PIN_6
#define Motor1_EA_HU_GPIO_Port GPIOC
#define Motor1_EB_HV_Pin GPIO_PIN_7
#define Motor1_EB_HV_GPIO_Port GPIOC
#define Motor1_EZ_HW_Pin GPIO_PIN_8
#define Motor1_EZ_HW_GPIO_Port GPIOC
#define Motor1_UH_Pin GPIO_PIN_8
#define Motor1_UH_GPIO_Port GPIOA
#define Motor1_VH_Pin GPIO_PIN_9
#define Motor1_VH_GPIO_Port GPIOA
#define Motor1_WH_Pin GPIO_PIN_10
#define Motor1_WH_GPIO_Port GPIOA
#define Motor1_SD_Pin GPIO_PIN_12
#define Motor1_SD_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
