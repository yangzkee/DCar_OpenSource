/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   SPI2 初始化接口和外部句柄声明。
  *
  * 硬件平台 : STM32F407VETx
  * 开发环境 : Keil uVision / STM32CubeMX HAL
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * 主要功能 : SPI2 主机模式，用于 ICM20602 通信
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* main.h：提供 SPI_HandleTypeDef 和 HAL SPI 相关宏。 */
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* SPI2 句柄，定义于 spi.c，imu.c 通过该句柄访问 ICM20602。 */
extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
 * @brief  初始化 SPI2。
 * @param  无。
 * @retval 无。
 * @note   配置为主机、双线、8 bit、Mode 3、软件 NSS。
 */
void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
