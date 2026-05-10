/**
  ******************************************************************************
  * @file    ps2_receiver.h
  * @brief   PS2 遥控器信号接收模块
  *          通过 PA3(USART2_RX) 接收 SBUS 信号，解析后直接存入变量 (0~2047)
  *
  * 硬件平台 : STM32F407VETx，USART2_RX 遥控输入
  * 开发环境 : Keil uVision / MDK-ARM
  * 晶振频率 : HSE 8 MHz，SYSCLK 168 MHz
  * 作者     : DCar_OpenSource
  * 日期     : 2026-05-10
  * 版本     : v1.0
  * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
  ******************************************************************************
  */
#ifndef __PS2_RECEIVER_H__
#define __PS2_RECEIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* main.h：提供 USART2 GPIO 宏、HAL 类型、uint8_t/uint16_t 等基础类型。 */
#include "main.h"

/* PS2/SBUS 数据接收引脚：PA3 (USART2_RX，定义于 main.h)。
 * 注意：SBUS 常见电平为反相串口，若接收机输出为反相电平而硬件未做反相处理，
 * MCU 可能无法解析到有效帧。请以实际接收机模块电平为准。
 */

/**
  * @brief  SBUS 摇杆数据结构 (保留 SBUS 原始范围 0~2047)
  * @note   SBUS 通道为 11 bit 数据，理论范围 0~2047，中位约 1024。
  *         本结构只保存控制逻辑关心的通道和连接状态。
  */
typedef struct {
  uint16_t lx;           /* 左摇杆 X 轴 (CH4) 0~2047 */
  uint16_t ly;           /* 左摇杆 Y 轴 (CH3) 0~2047 */
  uint16_t rx;           /* 右摇杆 X 轴 (CH1) 0~2047 */
  uint16_t ry;           /* 右摇杆 Y 轴 (CH2) 0~2047 */
  uint16_t ch6;          /* SBUS CH6 (channels[5]) 用于 O 键等 0~2047 */
  uint16_t buttons;      /* 按键位图 */
  uint8_t  connected;    /* 连接状态: 0=未连接, 1=已连接 */
} PS2_Data_TypeDef;

/**
  * @brief  PS2 接收模块初始化
  * @param  无。
  * @retval 无。
  * @note   初始化 SBUS 解析状态，启动 USART2 接收
  *         调用前需完成 MX_USART2_UART_Init()。
  */
void PS2_Receiver_Init(void);

/**
  * @brief  解析单个 SBUS 字节 (由 UART 接收回调调用)
  * @param  byte: 接收到的 1 字节串口数据，范围 0x00~0xFF。
  * @retval 无。
  * @note   协议帧格式通常为 25 字节：
  *         [0]=0x0F 帧头，[1..22]=16 路 11bit 通道打包数据，
  *         [23]=标志位，[24]=0x00 帧尾。
  *         WARNING：该函数运行在串口接收链路中，逻辑应保持短小。
  */
void PS2_Receiver_ParseByte(uint8_t byte);

/**
  * @brief  获取当前 PS2 数据 (预留接口)
  * @param  data: 指向 PS2_Data_TypeDef 的指针，用于接收数据，不能为 NULL。
  * @retval 无。
  * @note   调用者获得的是最近一次解析成功的数据快照。
  *         若在中断和主循环之间共享，必要时应考虑临界区保护。
  */
void PS2_Receiver_GetData(PS2_Data_TypeDef *data);

#ifdef __cplusplus
}
#endif

#endif /* __PS2_RECEIVER_H__ */
