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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIP_1_Pin GPIO_PIN_13
#define DIP_1_GPIO_Port GPIOC
#define DIP_2_Pin GPIO_PIN_14
#define DIP_2_GPIO_Port GPIOC
#define DIP_3_Pin GPIO_PIN_15
#define DIP_3_GPIO_Port GPIOC
#define VM_SENSE_Pin GPIO_PIN_0
#define VM_SENSE_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_3
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_4
#define LED_B_GPIO_Port GPIOA
#define ENC_EN_Pin GPIO_PIN_0
#define ENC_EN_GPIO_Port GPIOB
#define IFA_Pin GPIO_PIN_1
#define IFA_GPIO_Port GPIOB
#define IFA_EXTI_IRQn EXTI1_IRQn
#define IFB_Pin GPIO_PIN_2
#define IFB_GPIO_Port GPIOB
#define DRV_NFAULT_Pin GPIO_PIN_13
#define DRV_NFAULT_GPIO_Port GPIOB
#define DRV_EN_Pin GPIO_PIN_14
#define DRV_EN_GPIO_Port GPIOB
#define DRV_CAL_Pin GPIO_PIN_15
#define DRV_CAL_GPIO_Port GPIOB
#define INLX_Pin GPIO_PIN_6
#define INLX_GPIO_Port GPIOC
#define INHC_Pin GPIO_PIN_8
#define INHC_GPIO_Port GPIOA
#define INHB_Pin GPIO_PIN_9
#define INHB_GPIO_Port GPIOA
#define INHA_Pin GPIO_PIN_10
#define INHA_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
