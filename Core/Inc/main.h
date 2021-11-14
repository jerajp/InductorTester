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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint32_t PulseCounter;
extern uint32_t InductorNotConnected;
extern TIM_HandleTypeDef htim1;
extern uint32_t watch1,watch2,watch3,watch4;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define TEST1_ON  GPIOC->BSRR|=0x00004000  		//C14  1 << (14)
#define TEST1_OFF GPIOC->BSRR|=0x40000000     	//C14  1 << (16+14)
#define COMP_PIN_READ GPIOB->IDR&0x00000020
#define TIM50MS	5000	//TIM1 count 10us

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define TEST1_Pin GPIO_PIN_14
#define TEST1_GPIO_Port GPIOC
#define COMP_IN_Pin GPIO_PIN_5
#define COMP_IN_GPIO_Port GPIOB
#define COMP_IN_EXTI_IRQn EXTI9_5_IRQn
#define MOS_GATE_Pin GPIO_PIN_6
#define MOS_GATE_GPIO_Port GPIOB
#define LCD_CLK_Pin GPIO_PIN_7
#define LCD_CLK_GPIO_Port GPIOB
#define LCD_DIO_Pin GPIO_PIN_8
#define LCD_DIO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
