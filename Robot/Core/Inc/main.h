/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define ECHO_Pin GPIO_PIN_0
#define ECHO_GPIO_Port GPIOC
#define TRIGGER_Pin GPIO_PIN_1
#define TRIGGER_GPIO_Port GPIOC
#define DETECT5_Pin GPIO_PIN_2
#define DETECT5_GPIO_Port GPIOC
#define L2_Pin GPIO_PIN_3
#define L2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define P1_Pin GPIO_PIN_5
#define P1_GPIO_Port GPIOA
#define C_Pin GPIO_PIN_5
#define C_GPIO_Port GPIOC
#define ButtonExt_Pin GPIO_PIN_2
#define ButtonExt_GPIO_Port GPIOB
#define P2_Pin GPIO_PIN_10
#define P2_GPIO_Port GPIOB
#define G_Pin GPIO_PIN_11
#define G_GPIO_Port GPIOB
#define F_Pin GPIO_PIN_12
#define F_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOB
#define B_Pin GPIO_PIN_6
#define B_GPIO_Port GPIOC
#define A_Pin GPIO_PIN_8
#define A_GPIO_Port GPIOC
#define L1_Pin GPIO_PIN_8
#define L1_GPIO_Port GPIOA
#define E_Pin GPIO_PIN_11
#define E_GPIO_Port GPIOA
#define D_Pin GPIO_PIN_12
#define D_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DETECT3_Pin GPIO_PIN_15
#define DETECT3_GPIO_Port GPIOA
#define DETECT1_Pin GPIO_PIN_10
#define DETECT1_GPIO_Port GPIOC
#define DETECT2_Pin GPIO_PIN_12
#define DETECT2_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DETECT4_Pin GPIO_PIN_7
#define DETECT4_GPIO_Port GPIOB
#define DETECT3L_Pin GPIO_PIN_8
#define DETECT3L_GPIO_Port GPIOB
#define DETECT3P_Pin GPIO_PIN_9
#define DETECT3P_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
