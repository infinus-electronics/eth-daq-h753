/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define DUT_HVDC_ENABLE_Pin GPIO_PIN_2
#define DUT_HVDC_ENABLE_GPIO_Port GPIOE
#define DUT_VGS_IDLE_SEL_Pin GPIO_PIN_3
#define DUT_VGS_IDLE_SEL_GPIO_Port GPIOE
#define DUT_VICTRL_SEL_Pin GPIO_PIN_4
#define DUT_VICTRL_SEL_GPIO_Port GPIOE
#define DUT_GATE_SEL_Pin GPIO_PIN_5
#define DUT_GATE_SEL_GPIO_Port GPIOE
#define GADC_RESET_Pin GPIO_PIN_10
#define GADC_RESET_GPIO_Port GPIOD
#define GADC_RVS_Pin GPIO_PIN_11
#define GADC_RVS_GPIO_Port GPIOD
#define DUT_DAC_LDAC_Pin GPIO_PIN_14
#define DUT_DAC_LDAC_GPIO_Port GPIOD
#define DUT_DAC_RESET_Pin GPIO_PIN_15
#define DUT_DAC_RESET_GPIO_Port GPIOD
#define HS_ADC_START_Pin GPIO_PIN_8
#define HS_ADC_START_GPIO_Port GPIOA
#define HS_ADC_DRDY_Pin GPIO_PIN_9
#define HS_ADC_DRDY_GPIO_Port GPIOA
#define HS_ADC_RESET_Pin GPIO_PIN_10
#define HS_ADC_RESET_GPIO_Port GPIOA
#define EFUSE_FLT_Pin GPIO_PIN_8
#define EFUSE_FLT_GPIO_Port GPIOB
#define EFUSE_PGOOD_Pin GPIO_PIN_9
#define EFUSE_PGOOD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
