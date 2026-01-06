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

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

/* PA0 – czujnik Halla */
#define hall_Pin        GPIO_PIN_0
#define hall_GPIO_Port  GPIOA

/* UART VCP */
#define VCP_TX_Pin      GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin      GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA

/* OLED (SPI / sterowanie) */
#define RES_Pin         GPIO_PIN_4
#define RES_GPIO_Port   GPIOA
#define CS_Pin          GPIO_PIN_0
#define CS_GPIO_Port    GPIOB
#define DC_Pin          GPIO_PIN_1
#define DC_GPIO_Port    GPIOB

/* Przycisk ekranu – D9 = PA8 */
#define Switch_Pin          GPIO_PIN_8
#define Switch_GPIO_Port    GPIOA
#define Switch_EXTI_IRQn    EXTI9_5_IRQn

/* SWD + LED */
#define SWDIO_Pin       GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin       GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LD3_Pin         GPIO_PIN_3
#define LD3_GPIO_Port   GPIOB

/* USER CODE BEGIN Private defines */
/* Uproszczone aliasy na przycisk */
#define BTN_PORT    Switch_GPIO_Port
#define BTN_PIN     Switch_Pin
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
