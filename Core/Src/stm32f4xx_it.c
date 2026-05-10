/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   STM32F407 中断服务函数实现。
  *
  * 本文件由 STM32CubeMX 生成并加入用户回调逻辑。主要负责 Cortex-M4
  * 内核异常、USART1/USART2、TIM6_DAC 和 DMA2_Stream2 中断入口，再转交
  * HAL 中断处理函数触发对应 HAL 回调。
  *
  * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
  * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * 中断优先级 : 由 DCar_OpenSource.ioc 配置，NVIC_PRIORITYGROUP_4；
  *              USART1/USART2/TIM6/DMA2_Stream2 抢占优先级均为 0，SysTick 为 15
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
/* main.h          : HAL 总头文件、GPIO 宏和 Error_Handler() 声明。
 * stm32f4xx_it.h  : 本文件中断函数原型声明，需与启动文件向量表名称一致。
 */
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief  NMI 不可屏蔽中断服务函数。
 * @param  无。
 * @retval 无。
 * @note   中断向量：NonMaskableInt_IRQn。触发条件包括外部 NMI 引脚或时钟安全系统等。
 *         当前实现进入死循环，便于调试定位严重硬件异常。
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief  HardFault 硬件错误异常服务函数。
 * @param  无。
 * @retval 无。
 * @note   中断向量：HardFault_IRQn。常见触发条件包括非法地址访问、栈溢出、
 *         未对齐访问升级等。当前实现死循环等待调试器分析 SCB fault 寄存器。
 */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief  SysTick 系统节拍中断服务函数。
 * @param  无。
 * @retval 无。
 * @note   中断向量：SysTick_IRQn，优先级 15。触发周期 1 ms，用于 HAL_GetTick()
 *         和 loop.c 软件分频调度的时间基准。
 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief  USART1 全局中断服务函数。
 * @param  无。
 * @retval 无。
 * @note   中断向量：USART1_IRQn，触发条件包括 IDLE、错误等 UART 事件。
 *         HAL_UART_IRQHandler() 会进一步触发 HAL_UARTEx_RxEventCallback()
 *         或 HAL_UART_ErrorCallback()，用于上位机协议 DMA+IDLE 接收。
 */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief  USART2 全局中断服务函数。
 * @param  无。
 * @retval 无。
 * @note   中断向量：USART2_IRQn，触发条件为 SBUS 单字节接收完成或 UART 错误。
 *         HAL_UART_IRQHandler() 会触发 HAL_UART_RxCpltCallback() 解析 PS2/SBUS 字节。
 */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
 * @brief  TIM6_DAC 全局中断服务函数。
 * @param  无。
 * @retval 无。
 * @note   中断向量：TIM6_DAC_IRQn。当前 TIM6 配置 Prescaler=83、Period=999，
 *         在 APB1 定时器时钟 84 MHz 下产生 1 kHz 更新中断。
 *         HAL_TIM_IRQHandler() 会触发 HAL_TIM_PeriodElapsedCallback()。
 */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
 * @brief  DMA2 Stream2 全局中断服务函数。
 * @param  无。
 * @retval 无。
 * @note   中断向量：DMA2_Stream2_IRQn。当前绑定 USART1_RX，方向为外设到内存。
 *         HAL_DMA_IRQHandler() 处理 DMA 传输事件，并与 USART1 DMA+IDLE 接收协同工作。
 */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
 * @brief  HAL TIM 周期溢出回调。
 * @param  htim：触发回调的定时器句柄。
 * @retval 无。
 * @note   当前只处理 TIM6。TIM6 每 1 ms 溢出一次，tick_10ms 累计到 10 后
 *         调用 MotorControl_Update()，形成 100 Hz 电机速度 PID 闭环。
 *         WARNING：该回调在中断上下文执行，不应加入 printf、HAL_Delay()
 *         或长时间阻塞操作。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t tick_10ms = 0;
  if (htim->Instance == TIM6) {
    tick_10ms++;
    if (tick_10ms >= 10) {
      tick_10ms = 0;
      MotorControl_Update();   /* PID 速度环 */
    }
  }
}
/* USER CODE END 1 */
