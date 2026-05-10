/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用 PID 控制器模块，用于电机速度环、航向环和位置环。
  *
  * 硬件平台 : STM32F407VETx，ARM Cortex-M4F/FPU
  * 开发环境 : Keil uVision / MDK-ARM
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * 作者     : DCar_OpenSource
  * 日期     : 2026-05-10
  * 版本     : v1.0
  * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
  ******************************************************************************
  */
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* main.h：提供 HAL 工程基础类型和编译器兼容宏。 */
#include "main.h"

/**
 * @brief  PID 控制器状态结构体。
 * @note   Kp/Ki/Kd 为控制参数；integral 和 last_error 为运行时状态；
 *         out_max/out_min 为输出限幅；integral_max 用于抗积分饱和。
 */
typedef struct {
  float Kp;           /* 比例系数 */
  float Ki;           /* 积分系数 */
  float Kd;           /* 微分系数 */
  float integral;     /* 积分累积 */
  float last_error;   /* 上次误差 */
  float out_max;      /* 输出上限 */
  float out_min;      /* 输出下限 */
  float integral_max; /* 积分限幅，抗饱和 */
} PID_TypeDef;

/**
  * @brief  PID 参数初始化
  * @param  pid：PID 结构体指针。
  * @param  kp：比例增益。
  * @param  ki：积分增益。
  * @param  kd：微分增益。
  * @param  out_max：输出上限 (如 PWM 最大值 999)。
  * @param  out_min：输出下限 (如 -999 支持反转)。
  * @retval 无。
  */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float out_min);

/**
  * @brief  PID 计算
  * @param  pid：PID 结构体指针。
  * @param  target：目标值。
  * @param  current：当前反馈值。
  * @retval PID 输出，已限幅。
  */
float PID_Calc(PID_TypeDef *pid, float target, float current);

/**
  * @brief  重置 PID 内部状态
  * @param  pid：PID 结构体指针。
  * @retval 无。
  */
void PID_Reset(PID_TypeDef *pid);

/**
  * @brief  设置积分限幅，防止积分饱和
  * @param  pid：PID 结构体指针。
  * @param  limit：积分限幅绝对值。
  * @retval 无。
  */
void PID_SetIntegralLimit(PID_TypeDef *pid, float limit);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
