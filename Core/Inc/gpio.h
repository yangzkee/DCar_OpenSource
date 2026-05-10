/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   GPIO 初始化接口声明。
  *
  * 硬件平台 : STM32F407VETx
  * 开发环境 : Keil uVision / STM32CubeMX HAL
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * 主要功能 : 声明 MX_GPIO_Init()，配置 LED、ICM20602 CS 和电机方向 GPIO
  * 作者     : DCar_OpenSource
  * 日期     : 2026-05-10
  * 版本     : v1.0
  * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* main.h：提供 GPIO 引脚宏、GPIO_TypeDef、GPIO_PinState 和 HAL GPIO API。 */
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
 * @brief  初始化普通 GPIO 引脚。
 * @param  无。
 * @retval 无。
 * @note   实现位于 gpio.c。复用功能 GPIO 由对应外设 MSP 初始化函数配置。
 */
void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */
