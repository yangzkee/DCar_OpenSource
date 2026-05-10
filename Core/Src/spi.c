/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   SPI2 外设初始化与 MSP GPIO 配置。
  *
  * 当前工程仅使用 SPI2 与 ICM20602 六轴 IMU 通信。片选 CS 由 gpio.c 配置
  * 的 PA8 软件控制，不使用硬件 NSS。
  *
  * 硬件平台 : STM32F407VETx
  * 开发环境 : Keil uVision / STM32CubeMX HAL
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * SPI 引脚  : PB13=SCK，PB14=MISO，PB15=MOSI，PA8=软件 CS
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
/* spi.h：声明 hspi2 和 MX_SPI2_Init()，并间接包含 HAL SPI 类型。 */
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* SPI2 HAL 句柄。
 * imu.c 通过该句柄调用 HAL_SPI_TransmitReceive() 读写 ICM20602。
 */
SPI_HandleTypeDef hspi2;

/**
 * @brief  初始化 SPI2 为主机模式。
 * @param  无。
 * @retval 无。
 * @note   配置要点：
 *         - 主机、双线全双工、8 bit 数据宽度；
 *         - CPOL=1、CPHA=2EDGE，对应 SPI Mode 3，匹配 ICM20602 常用 SPI 模式；
 *         - 软件 NSS，片选由 PA8 手动控制；
 *         - Prescaler=2，SPI2 挂在 APB1，当前 PCLK1=42 MHz，SCK 约 21 MHz。
 *         WARNING：若 ICM20602 供电或走线质量较差，21 MHz 可能过高，可适当增大分频。
 */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  /* SPI_CR1 MSTR=1：主机模式，由 MCU 产生 SCK。 */
  hspi2.Init.Mode = SPI_MODE_MASTER;
  /* 双线全双工：MOSI/MISO 同时传输，适合寄存器读写。 */
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
 * @brief  SPI 底层 MSP 初始化，配置 SPI2 时钟和 GPIO 复用。
 * @param  spiHandle：HAL SPI 句柄。
 * @retval 无。
 * @note   PB13/PB14/PB15 配置为 AF5_SPI2、推挽复用、无上下拉、很高速度。
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

/**
 * @brief  SPI 底层 MSP 反初始化。
 * @param  spiHandle：HAL SPI 句柄。
 * @retval 无。
 * @note   关闭 SPI2 时钟并释放 PB13/PB14/PB15 复用配置。
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
