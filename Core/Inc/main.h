/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : main.c 公共头文件，包含应用全局 GPIO 定义和错误处理入口。
 *
 * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* stm32f4xx_hal.h：
 * STM32F4 HAL 总入口头文件，提供 GPIO/UART/SPI/TIM/DMA/RCC 等外设类型、
 * 寄存器位宏、HAL_StatusTypeDef、GPIO_PinState 以及 HAL_xxx API 声明。
 */
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#if defined(__GNUC__) && !defined(__CC_ARM)
/* GCC 兼容宏。
 * Keil/ARMCC 中常见的 __weak、__packed 关键字在 GCC 下名称不同，
 * 这里映射到 GCC attribute，便于同一份源码在 Makefile/GCC 环境下编译。
 */
#ifndef __weak
#define __weak __attribute__((weak))
#endif
#ifndef __packed
#define __packed __attribute__((packed))
#endif
#endif
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
 * @brief  系统错误处理入口。
 * @param  无。
 * @retval 无。
 * @note   由 HAL 初始化或运行时错误路径调用；实现位于 main.c。
 */
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* LED0 指示灯引脚。
 * 引脚：PE5。当前代码中 GPIO_PIN_RESET 表示点亮，推测 LED 为低电平有效。
 * WARNING：不同开发板 LED 极性可能不同，移植时需实测确认。
 */
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOE

/* ICM20602 SPI 片选引脚。
 * 引脚：PA8，低电平有效，由 imu.c 的 ICM_CS_LOW/HIGH 宏手动控制。
 */
#define CS_ICM_Pin GPIO_PIN_8
#define CS_ICM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
/* 电机方向控制引脚 - 放在此处 CubeMX 重新生成不会删除。
 * A/B/C/D 对应四个电机，IN1/IN2 为 H 桥方向控制输入，均位于 GPIOD。
 * 逻辑由 motor_control.c 统一定义：
 * - IN1=1、IN2=0：正向；
 * - IN1=0、IN2=1：反向；
 * - IN1=0、IN2=0：空载滑行。
 */
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOD
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOD
#define BIN1_Pin GPIO_PIN_2
#define BIN1_GPIO_Port GPIOD
#define BIN2_Pin GPIO_PIN_3
#define BIN2_GPIO_Port GPIOD
#define CIN1_Pin GPIO_PIN_4
#define CIN1_GPIO_Port GPIOD
#define CIN2_Pin GPIO_PIN_5
#define CIN2_GPIO_Port GPIOD
#define DIN1_Pin GPIO_PIN_6
#define DIN1_GPIO_Port GPIOD
#define DIN2_Pin GPIO_PIN_7
#define DIN2_GPIO_Port GPIOD

/* PS2/SBUS 接收引脚。
 * 引脚：PA3，复用为 USART2_RX。usart.c 中 USART2 配置为 SBUS 常用格式：
 * 38400 baud、偶校验、2 stop bits、9 bit word length。
 */
#define PS2_DATA_Pin GPIO_PIN_3
#define PS2_DATA_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
