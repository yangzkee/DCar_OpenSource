/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   定时器外设接口声明。
  *
  * 本文件由 CubeMX 生成基础结构，补充中文注释用于说明各 TIM 句柄与
  * 小车硬件功能的对应关系：
  *   - TIM2/TIM3/TIM4/TIM5：四路 AB 相编码器接口；
  *   - TIM6：1 ms 基本定时中断，软件分频后触发 10 ms 电机 PID；
  *   - TIM8：四路 PWM 输出，驱动 M1~M4 电机调速。
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* main.h：提供 HAL 基础定义、GPIO 引脚宏、错误处理函数以及 STM32F4xx 设备头。 */
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* TIM2 编码器句柄。
 * 硬件关联：M1 编码器 A/B 相，PA5 / PB3。
 * 计数模式：TIM_ENCODERMODE_TI12，双通道正交计数。
 */
extern TIM_HandleTypeDef htim2;

/* TIM3 编码器句柄。
 * 硬件关联：M2 编码器 A/B 相，PA6 / PA7。
 */
extern TIM_HandleTypeDef htim3;

/* TIM4 编码器句柄。
 * 硬件关联：M3 编码器 A/B 相，PB6 / PB7。
 */
extern TIM_HandleTypeDef htim4;

/* TIM5 编码器句柄。
 * 硬件关联：M4 编码器 A/B 相，PA0 / PA1。
 */
extern TIM_HandleTypeDef htim5;

/* TIM6 基本定时器句柄。
 * 用途：1 ms 周期中断，在 TIM6_DAC_IRQHandler()/HAL 回调链路中进行软件计数，
 *       每 10 次调用一次 MotorControl_Update()，形成 100 Hz 速度闭环。
 * WARNING：TIM6 无输入输出通道，仅用于时间基准；不要在中断中加入阻塞操作。
 */
extern TIM_HandleTypeDef htim6;

/* TIM8 PWM 句柄。
 * 硬件关联：PC6~PC9 对应 TIM8_CH1~CH4，输出四路电机 PWM。
 * PWM 周期：Period=999 时，一个 PWM 周期含 1000 个计数，比较值 0~999。
 */
extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @brief  初始化 TIM2 编码器接口。
  * @param  无。
  * @retval 无。
  * @note   仅完成寄存器和 GPIO 复用配置；真正开始计数需调用 HAL_TIM_Encoder_Start()。
  */
void MX_TIM2_Init(void);

/**
  * @brief  初始化 TIM3 编码器接口。
  * @param  无。
  * @retval 无。
  * @note   用于 M2 编码器，PA6/PA7 配置为 TIM3 复用输入。
  */
void MX_TIM3_Init(void);

/**
  * @brief  初始化 TIM4 编码器接口。
  * @param  无。
  * @retval 无。
  * @note   用于 M3 编码器，PB6/PB7 配置为 TIM4 复用输入。
  */
void MX_TIM4_Init(void);

/**
  * @brief  初始化 TIM5 编码器接口。
  * @param  无。
  * @retval 无。
  * @note   用于 M4 编码器，PA0/PA1 配置为 TIM5 复用输入。
  */
void MX_TIM5_Init(void);

/**
  * @brief  初始化 TIM6 基本定时器。
  * @param  无。
  * @retval 无。
  * @note   本工程将 TIM6 作为 1 ms 系统调度节拍之一；启动中断需额外调用
  *         HAL_TIM_Base_Start_IT(&htim6)。
  */
void MX_TIM6_Init(void);

/**
  * @brief  初始化 TIM8 四通道 PWM。
  * @param  无。
  * @retval 无。
  * @note   CH1~CH4 分别输出到 PC6/PC7/PC8/PC9；启动 PWM 需额外调用
  *         HAL_TIM_PWM_Start()。
  */
void MX_TIM8_Init(void);

/**
  * @brief  TIM PWM GPIO 后初始化钩子。
  * @param  htim：定时器句柄指针，当前主要处理 TIM8 PWM 输出引脚复用。
  * @retval 无。
  * @note   CubeMX 在 MX_TIM8_Init() 内部调用该函数配置 PC6~PC9 的 AF3。
  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
