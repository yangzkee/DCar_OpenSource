/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   STM32F4 中断服务函数声明。
  *
  * 本文件声明 Cortex-M4 内核异常和片上外设中断入口。具体实现位于
  * stm32f4xx_it.c，中断向量表由启动文件 startup_stm32f407xx.s 引用。
  *
  * 本工程重点外设中断：
  *   - SysTick          ：HAL 系统节拍；
  *   - USART1/USART2    ：上位机控制和遥控接收；
  *   - DMA2_Stream2     ：USART1_RX DMA 接收；
  *   - TIM6_DAC         ：1 ms 定时基准，软件分频为 10 ms 电机控制节拍。
  *
  * 硬件平台 : STM32F407VETx，ARM Cortex-M4F
  * 开发环境 : Keil uVision / MDK-ARM
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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

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
/**
  * @brief  不可屏蔽中断 NMI 入口。
  * @param  无。
  * @retval 无。
  * @note   中断向量：NMI_IRQn；优先级固定，仅次于 Reset。
  */
void NMI_Handler(void);

/**
  * @brief  硬Fault异常入口。
  * @param  无。
  * @retval 无。
  * @note   触发条件包括非法取指、总线访问异常升级、未处理异常等。
  *         WARNING：发生 HardFault 时通常说明存在严重指针、栈或外设访问问题。
  */
void HardFault_Handler(void);

/**
  * @brief  存储器管理异常入口。
  * @param  无。
  * @retval 无。
  * @note   Cortex-M4 MPU 相关异常；若未启用 MPU，一般较少触发。
  */
void MemManage_Handler(void);

/**
  * @brief  总线Fault异常入口。
  * @param  无。
  * @retval 无。
  * @note   常见于访问非法地址、外设时钟未使能却访问寄存器等情况。
  */
void BusFault_Handler(void);

/**
  * @brief  用法Fault异常入口。
  * @param  无。
  * @retval 无。
  * @note   常见于未对齐访问、除零、执行未定义指令等内核异常。
  */
void UsageFault_Handler(void);

/**
  * @brief  SVC 系统服务调用异常入口。
  * @param  无。
  * @retval 无。
  * @note   裸机工程通常不主动使用；若接入 RTOS，可能由内核调度使用。
  */
void SVC_Handler(void);

/**
  * @brief  调试监控异常入口。
  * @param  无。
  * @retval 无。
  * @note   与断点、单步调试等调试事件相关。
  */
void DebugMon_Handler(void);

/**
  * @brief  PendSV 异常入口。
  * @param  无。
  * @retval 无。
  * @note   裸机工程通常空实现；RTOS 中常用于上下文切换。
  */
void PendSV_Handler(void);

/**
  * @brief  SysTick 系统节拍中断入口。
  * @param  无。
  * @retval 无。
  * @note   中断向量：SysTick_IRQn；由 HAL_IncTick() 维护 HAL 毫秒计时。
  */
void SysTick_Handler(void);

/**
  * @brief  USART1 全局中断入口。
  * @param  无。
  * @retval 无。
  * @note   中断向量：USART1_IRQn；触发条件包括 IDLE 空闲线、接收错误、发送完成等。
  *         本工程主要用于 DMA 接收空闲帧判定，上位机控制协议在 usart1_control.c 中解析。
  */
void USART1_IRQHandler(void);

/**
  * @brief  USART2 全局中断入口。
  * @param  无。
  * @retval 无。
  * @note   中断向量：USART2_IRQn；用于遥控/SBUS 字节接收和错误处理。
  */
void USART2_IRQHandler(void);

/**
  * @brief  TIM6/DAC 共用中断入口。
  * @param  无。
  * @retval 无。
  * @note   中断向量：TIM6_DAC_IRQn；TIM6 更新事件触发。
  *         本工程利用 TIM6 作为 1 ms 基准，并在 HAL_TIM_PeriodElapsedCallback()
  *         中进行软件计数，形成 100 Hz 电机控制周期。
  */
void TIM6_DAC_IRQHandler(void);

/**
  * @brief  DMA2 Stream2 中断入口。
  * @param  无。
  * @retval 无。
  * @note   中断向量：DMA2_Stream2_IRQn；硬件关联 USART1_RX。
  *         涉及寄存器：DMA2_LISR/DMA2_LIFCR 中的 TCIF2、HTIF2、TEIF2 等标志位。
  */
void DMA2_Stream2_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
