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
#include "stm32g0xx_hal.h"

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
struct button_t {

  uint32_t last_time_change;
  uint32_t first_time_change;
  uint32_t ring_period;
  uint8_t state;
  uint32_t count;
  uint8_t  last_pin;

};

struct signal_t{

	uint8_t state;
	uint32_t last_time_change;
	uint32_t period;
	uint32_t count;
};

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SIGNAL_READ_Pin GPIO_PIN_7
#define SIGNAL_READ_GPIO_Port GPIOB
#define SIGNAL_READ_EXTI_IRQn EXTI4_15_IRQn
#define PICK_UP_DET_Pin GPIO_PIN_0
#define PICK_UP_DET_GPIO_Port GPIOB
#define PICK_UP_DET_EXTI_IRQn EXTI0_1_IRQn
#define PICK_UP_OUT_Pin GPIO_PIN_12
#define PICK_UP_OUT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define SIGNAL_READ_Pin GPIO_PIN_7
#define SIGNAL_READ_GPIO_Port GPIOB
#define SIGNAL_READ_EXTI_IRQn EXTI4_15_IRQn
#define SIGNAL_WRITE_Pin GPIO_PIN_2
#define SIGNAL_WRITE_GPIO_Port GPIOF
#define PICK_UP_DET_Pin GPIO_PIN_0
#define PICK_UP_DET_GPIO_Port GPIOB
#define PICK_UP_DET_EXTI_IRQn EXTI0_1_IRQn
#define PICK_UP_OUT_Pin GPIO_PIN_12
#define PICK_UP_OUT_GPIO_Port GPIOA
#define OPEN_OUT_Pin GPIO_PIN_13
#define OPEN_OUT_GPIO_Port GPIOA
#define RING_DET_Pin GPIO_PIN_14
#define RING_DET_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
