/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   串口外设接口声明。
  *
  * 本文件声明 UART4、USART1、USART2 的 HAL 句柄和初始化函数：
  *   - UART4 ：115200 8N1，调试输出口；
  *   - USART1：115200 8N1，DMA2_Stream2 接收 + IDLE 空闲中断，用于上位机控制；
  *   - USART2：38400，9B/even/2 stop，用于 SBUS/PS2 遥控接收。
  *
  * 硬件平台 : STM32F407VETx，ARM Cortex-M4F
  * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
  * 晶振频率 : HSE 8 MHz，SYSCLK 168 MHz
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* main.h：提供 UART_HandleTypeDef、HAL 状态码、GPIO 引脚宏和 Error_Handler()。 */
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* UART4 调试串口句柄。
 * 用途：输出 IMU 姿态、调试文本等低速诊断信息。
 * WARNING：若在中断中直接发送，可能阻塞实时控制链路，建议只在主循环或低频任务使用。
 */
extern UART_HandleTypeDef huart4;

/* USART1 上位机控制串口句柄。
 * 用途：接收 0x67 速度控制帧和 0x64 位移控制帧。
 * 接收方式：DMA 循环/一次接收 + IDLE 空闲中断，DMA 通道为 DMA2_Stream2。
 */
extern UART_HandleTypeDef huart1;

/* USART2 遥控/SBUS 接收串口句柄。
 * 用途：接收 PS2/SBUS 25 字节帧，协议常用 100 kbaud 8E2；
 *       当前工程配置按 .ioc 为 38400、9B、偶校验、2 stop，请以实际遥控接收机为准。
 * WARNING：串口帧格式必须与接收机输出一致，否则会出现通道值不更新或大量帧错误。
 */
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @brief  初始化 UART4 调试串口。
  * @param  无。
  * @retval 无。
  * @note   仅完成 UART4 波特率、字长、停止位、GPIO 复用等硬件配置；
  *         调试发送函数在 uart4_debug.c 中封装。
  */
void MX_UART4_Init(void);

/**
  * @brief  初始化 USART1 上位机控制串口。
  * @param  无。
  * @retval 无。
  * @note   初始化后需由 Uart1_Control_Init() 启动 DMA+IDLE 接收。
  */
void MX_USART1_UART_Init(void);

/**
  * @brief  初始化 USART2 遥控接收串口。
  * @param  无。
  * @retval 无。
  * @note   初始化后由 PS2_Receiver_Init() 启动逐字节接收或解析状态机。
  */
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
