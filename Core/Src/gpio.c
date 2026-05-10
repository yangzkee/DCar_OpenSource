/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO 引脚初始化配置。
  *
  * 本文件配置普通 GPIO 输出/输入引脚，不包含复用功能引脚；TIM/USART/SPI
  * 的复用 GPIO 由各自外设 MSP 初始化函数配置。
  *
  * 硬件平台 : STM32F407VETx
  * 开发环境 : Keil uVision / STM32CubeMX HAL
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * 主要引脚 : PE5 LED0，PA8 ICM20602 片选，PD0~PD7 电机方向控制
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
/* gpio.h：声明 MX_GPIO_Init()，并间接包含 main.h 中的 GPIO 引脚宏定义。 */
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @brief  初始化普通 GPIO 引脚。
 * @param  无。
 * @retval 无。
 * @note   配置内容：
 *         - PE5：LED0，推挽输出，上拉，低速，默认输出 SET。
 *         - PA8：ICM20602 CS，推挽输出，无上下拉，高速，默认拉高取消片选。
 *         - PD0~PD7：四路电机 H 桥方向控制，推挽输出，上拉，高速，默认拉低。
 *         - PD9：输入，上拉，当前代码未直接使用，推测保留作按键/检测信号。
 *         WARNING：电机方向引脚上电默认拉低，配合 PWM=0 可避免电机误动作；
 *         若驱动板方向输入极性不同，应同步检查 motor_control.c。
 */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 使能用到的 GPIO 端口时钟。
   * RCC AHB1ENR 对应位被置 1 后，才能访问 GPIOx 寄存器。
   */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* LED0 默认输出高电平。
   * 当前工程中 LED0 低电平点亮，因此默认灭灯。
   */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

  /* 电机方向控制引脚默认拉低。
   * PD0~PD7 对应 A/B/C/D 四个电机的 IN1/IN2；PD10~PD12 为保留输出。
   */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /* ICM20602 CS 默认拉高，SPI 从机未选中。 */
  HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin, GPIO_PIN_SET);

  /* LED0：推挽输出，低速足够驱动指示灯，上拉可降低悬空风险。 */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /* PD9：上拉输入。当前业务代码未直接读取，推测为预留按键/限位输入。 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* GPIOD 输出组：
   * PD0~PD7：电机方向控制；
   * PD10~PD12：保留输出，默认复位低电平。
   */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* PA8 ICM20602 片选：软件控制，低电平有效，要求边沿干净，因此使用高速输出。 */
  GPIO_InitStruct.Pin = CS_ICM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_ICM_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
