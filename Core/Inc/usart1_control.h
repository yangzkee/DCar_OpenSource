/**
 ******************************************************************************
 * @file    usart1_control.h
 * @brief   上位机串口控制接口 (USART1 + DMA+IDLE)
 *
 * 支持两种控制帧:
 *   - 0x67 持续速度控制 (Vx/Vy/w, mm/s)
 *   - 0x64 定点匀速位移 (X/Y/Z cm, 速度 cm/s)
 *
 * 硬件平台 : STM32F407VETx，USART1 + DMA2_Stream2
 * 开发环境 : Keil uVision / MDK-ARM
 * 晶振频率 : HSE 8 MHz，SYSCLK 168 MHz
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 ******************************************************************************
 */
#ifndef __USART1_CONTROL_H__
#define __USART1_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* main.h：提供 UART/DMA HAL 类型、基础整数类型和工程公共宏。 */
#include "main.h"

/**
 * @brief  串口指令类型
 * @note   解析状态机在收到完整且校验通过的协议帧后写入该枚举。
 */
typedef enum {
  UART1_CMD_NONE = 0,     /* 无有效命令或命令已被处理 */
  UART1_CMD_VELOCITY,     /* 0x67 持续速度控制 */
  UART1_CMD_DISPLACEMENT /* 0x64 定点匀速位移 */
} Uart1_CmdType_t;

/**
 * @brief  串口控制命令结构体
 *         由 usart1_control.c 在收到完整一帧数据后填充
 * @note   单位约定：
 *         速度帧内部统一换算到 mm/s；
 *         位移帧内部统一换算到 mm；
 *         valid=1 表示命令尚未被主循环消费。
 */
typedef struct {
  Uart1_CmdType_t type; /* 指令类型 */

  /* 速度控制参数 (type == UART1_CMD_VELOCITY) */
  float vx_mmps; /* 车体 X 方向线速度 (mm/s)，正=右，负=左 */
  float vy_mmps; /* 车体 Y 方向线速度 (mm/s)，正=前，负=后 */
  float w_mmps;  /* 车体旋转等效线速度 (mm/s)，正=顺时针 */

  /* 位移控制参数 (type == UART1_CMD_DISPLACEMENT) */
  float target_x_mm;       /* 相对 X 位移 (mm) */
  float target_y_mm;       /* 相对 Y 位移 (mm) */
  float target_z_mm;       /* 相对 Z 位移 (mm)，暂存为 0 */
  float target_speed_mmps; /* 移动速度上限 (mm/s) */

  uint8_t valid; /* 当前命令是否有效 (1=有效，0=无效或已过期) */
} Uart1_ControlCmd_t;

/**
 * @brief  标记命令已处理，清除有效位
 * @param  无。
 * @retval 无。
 * @note   主循环读取并执行最新命令后调用，避免同一帧被重复执行。
 */
void Uart1_Control_ClearCmd(void);

/**
 * @brief  USART1 控制模块初始化
 * @param  无。
 * @retval 无。
 * @note   需在 MX_USART1_UART_Init 之后调用
 *         内部会启动 DMA+IDLE 接收
 *         WARNING：DMA 缓冲区由模块内部维护，不要在其他模块直接修改 USART1 DMA 状态。
 */
void Uart1_Control_Init(void);

/**
 * @brief  获取最近一次解析成功的控制命令
 * @param  out: 用户提供的结构体指针，用于接收数据
 * @retval 无。
 * @note   若 out 为 NULL，函数实现应直接返回；调用者根据 out->valid 判断是否执行。
 */
void Uart1_Control_GetLatestCmd(Uart1_ControlCmd_t *out);

/**
 * @brief  发送串口应答（在主循环中调用）
 * @param  无。
 * @retval 无。
 * @note   内部检查应答标志，有应答时才发送
 *         建议放在主循环或低频任务中，避免在中断中阻塞发送。
 */
void Uart1_Control_SendAck(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART1_CONTROL_H__ */
