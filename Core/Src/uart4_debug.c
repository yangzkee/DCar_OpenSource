/**
 ******************************************************************************
 * @file    uart4_debug.c
 * @brief   UART4 调试输出辅助函数。
 *
 * 本文件提供面向调试工具的文本输出接口。当前实现把 IMU 姿态角按照
 * CSV 格式通过 UART4 发出，便于串口助手或 VOFA+ 观察 roll/pitch/yaw。
 *
 * 硬件平台 : STM32F407VETx
 * 开发环境 : Keil uVision / STM32CubeMX HAL
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
 * 串口配置 : UART4，115200 baud，8N1
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 ******************************************************************************
 */

/* 头文件说明：
 * uart4_debug.h : 声明 UART4_Debug_SendIMUData()，并包含 IMU 姿态结构体类型。
 * usart.h       : 提供 huart4 句柄。
 * stdio.h       : 提供 snprintf()，用于格式化 CSV 字符串。
 */
#include "uart4_debug.h"
#include "usart.h"
#include <stdio.h>

/**
 * @brief  通过 UART4 发送 IMU 姿态角调试数据。
 * @param  angle：ICM20602_Attitude_TypeDef 指针，包含 roll/pitch/yaw，单位 degree。
 * @retval 无。
 * @note   输出格式为 "%.2f,%.2f,%.2f\n"，即 Roll,Pitch,Yaw，每行一组。
 *         HAL_UART_Transmit() 超时时间 10 ms。
 *         WARNING：该函数使用阻塞串口发送，不建议在高频中断中调用。
 */
void UART4_Debug_SendIMUData(ICM20602_Attitude_TypeDef *angle) {
  char buf[64];
  /* 采用 CSV 格式：Roll,Pitch,Yaw\n，方便 VOFA+ 或串口绘图工具解析。 */
  int len = snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f\n", angle->roll,
                     angle->pitch, angle->yaw);
  if (len > 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 10);
  }
}
