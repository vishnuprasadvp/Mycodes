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
#include "stm32f1xx_hal.h"

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
#define RELAY2_Pin GPIO_PIN_4
#define RELAY2_GPIO_Port GPIOE
#define RELAY3_Pin GPIO_PIN_5
#define RELAY3_GPIO_Port GPIOE
#define RELAY4_Pin GPIO_PIN_6
#define RELAY4_GPIO_Port GPIOE
#define INTERRUPT_Pin GPIO_PIN_0
#define INTERRUPT_GPIO_Port GPIOA
#define INTERRUPT_EXTI_IRQn EXTI0_IRQn
#define RELAY6_Pin GPIO_PIN_4
#define RELAY6_GPIO_Port GPIOC
#define RELAY5_Pin GPIO_PIN_5
#define RELAY5_GPIO_Port GPIOC
#define RELAY1_Pin GPIO_PIN_0
#define RELAY1_GPIO_Port GPIOB
#define RELAY0_Pin GPIO_PIN_1
#define RELAY0_GPIO_Port GPIOB
#define TOUCH9_Pin GPIO_PIN_8
#define TOUCH9_GPIO_Port GPIOE
#define LED8_Pin GPIO_PIN_9
#define LED8_GPIO_Port GPIOE
#define TOUCH0_Pin GPIO_PIN_10
#define TOUCH0_GPIO_Port GPIOE
#define indication_led_Pin GPIO_PIN_11
#define indication_led_GPIO_Port GPIOE
#define LED9_Pin GPIO_PIN_13
#define LED9_GPIO_Port GPIOE
#define LED10_Pin GPIO_PIN_14
#define LED10_GPIO_Port GPIOE
#define TOUCH2_Pin GPIO_PIN_12
#define TOUCH2_GPIO_Port GPIOB
#define TOUCH4_Pin GPIO_PIN_13
#define TOUCH4_GPIO_Port GPIOB
#define TOUCH1_Pin GPIO_PIN_14
#define TOUCH1_GPIO_Port GPIOB
#define TOUCH3_Pin GPIO_PIN_15
#define TOUCH3_GPIO_Port GPIOB
#define TOUCH5_Pin GPIO_PIN_8
#define TOUCH5_GPIO_Port GPIOD
#define DIMMER_DOWN_Pin GPIO_PIN_9
#define DIMMER_DOWN_GPIO_Port GPIOD
#define DIMMER_UP_Pin GPIO_PIN_10
#define DIMMER_UP_GPIO_Port GPIOD
#define TOUCH6_Pin GPIO_PIN_11
#define TOUCH6_GPIO_Port GPIOD
#define LED7_Pin GPIO_PIN_12
#define LED7_GPIO_Port GPIOD
#define LED6_Pin GPIO_PIN_13
#define LED6_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOD
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_7
#define LED5_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOC
#define DIMMER_Pin GPIO_PIN_15
#define DIMMER_GPIO_Port GPIOA
#define ESP_RST_Pin GPIO_PIN_0
#define ESP_RST_GPIO_Port GPIOD
#define STM_TX_Pin GPIO_PIN_5
#define STM_TX_GPIO_Port GPIOD
#define STM_RX_Pin GPIO_PIN_6
#define STM_RX_GPIO_Port GPIOD
#define TOUCH10_Pin GPIO_PIN_0
#define TOUCH10_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
