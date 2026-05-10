/**
 *******************************************************************************
 * @file    uart4_debug.h
 * @brief   UART4 调试输出接口。
 *
 * 本模块把 IMU 姿态数据格式化为可读字符串后通过 UART4 发送，便于在
 * Keil 调试、串口助手或上位机中观察姿态角、角速度等运行状态。
 *
 * 硬件平台 : STM32F407VETx，UART4 调试串口
 * 开发环境 : Keil uVision / MDK-ARM
 * 晶振频率 : HSE 8 MHz，SYSCLK 168 MHz
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 *******************************************************************************
 */
#ifndef __UART4_DEBUG_H__
#define __UART4_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* imu.h：提供 ICM20602_Attitude_TypeDef 姿态数据结构体定义。 */
#include "imu.h"
/* main.h：提供 HAL UART 类型、基础整数类型和工程公共宏。 */
#include "main.h"

/**
 * @brief  通过 UART4 发送 IMU 姿态数据字符串。
 * @param  angle：指向 ICM20602_Attitude_TypeDef 的姿态数据指针，不能为 NULL；
 *                结构体中通常包含 roll/pitch/yaw 等解算结果。
 * @retval 无。
 * @note   该函数通常用于调试输出，内部可能调用 HAL_UART_Transmit() 等阻塞式发送。
 *         WARNING：不要在高频中断或电机 PID 中断中频繁调用，否则会拉长中断执行时间，
 *         造成控制周期抖动。建议在主循环低频任务中调用。
 */
void UART4_Debug_SendIMUData(ICM20602_Attitude_TypeDef *angle);

#ifdef __cplusplus
}
#endif

#endif /* __UART4_DEBUG_H__ */
