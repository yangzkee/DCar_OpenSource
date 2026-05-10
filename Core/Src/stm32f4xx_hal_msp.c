/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        HAL MSP 全局初始化代码。
  *
  * MSP 是 MCU Support Package 的缩写，负责 HAL 外设初始化前的底层资源
  * 准备。本文件只包含全局 MSP 初始化；具体 UART/SPI/TIM 的 MSP 初始化
  * 已由 CubeMX 分散生成在 usart.c、spi.c、tim.c 中。
  *
  * 硬件平台 : STM32F407VETx
  * 开发环境 : Keil uVision / STM32CubeMX HAL
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
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

/* Includes ------------------------------------------------------------------*/
/* main.h：提供 HAL 基础类型、RCC 宏和全局工程定义。 */
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * @brief  初始化 HAL 全局 MSP 资源。
 * @param  无。
 * @retval 无。
 * @note   使能 SYSCFG 和 PWR 外设时钟：
 *         - SYSCFG：用于外部中断映射、部分系统配置功能；
 *         - PWR：用于电源控制、电压调节、低功耗等功能。
 *         本函数由 HAL_Init() 间接调用，早于各外设 MX_xxx_Init()。
 */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init。
   * 当前系统异常优先级大多保持 CubeMX 默认值；外设中断在各 MSP 文件中配置。
   */

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */
/* HAL_UART_MspInit 已由 CubeMX 在 usart.c 中生成。
 * SPI/TIM 的 MSP 初始化也分别位于 spi.c、tim.c。
 */
/* USER CODE END 1 */
