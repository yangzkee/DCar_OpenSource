/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   DMA 控制器时钟和中断初始化。
  *
  * 当前工程使用 DMA2_Stream2 接收 USART1_RX 数据，配合 USART1 IDLE 事件
  * 实现上位机控制协议的变长帧接收。
  *
  * 硬件平台 : STM32F407VETx
  * 开发环境 : Keil uVision / STM32CubeMX HAL
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * DMA 映射 : USART1_RX -> DMA2_Stream2 / Channel 4
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
/* dma.h：声明 MX_DMA_Init()，并包含 HAL DMA 相关类型和宏。 */
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @brief  使能 DMA 控制器时钟并配置 DMA2_Stream2 中断。
 * @param  无。
 * @retval 无。
 * @note   这里只配置 DMA 控制器公共部分；USART1_RX 的具体 Stream、Channel、
 *         方向、地址递增和数据宽度在 usart.c 的 HAL_UART_MspInit() 中配置。
 *         中断优先级设置为抢占优先级 0、子优先级 0。
 */
void MX_DMA_Init(void)
{

  /* 使能 DMA2 控制器时钟。
   * USART1_RX 位于 DMA2_Stream2，因此必须先打开 DMA2 时钟。
   */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration。
   * 触发条件包括传输完成、传输错误等，由 HAL_DMA_IRQHandler() 统一处理。
   */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
