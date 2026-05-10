/**
  ******************************************************************************
  * @file    pid.c
  * @brief   通用 PID 控制器实现。
  *
  * 本文件提供一个浮点 PID 控制器，当前用于电机速度环、yaw 航向保持和
  * 位置环。采样周期默认取 ENCODER_SAMPLE_MS，因此调用方需要按固定周期
  * 调用 PID_Calc() 才能保持积分和微分项量纲正确。
  *
  * 硬件平台 : STM32F407VETx，ARM Cortex-M4F/FPU
  * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * 作者     : DCar_OpenSource
  * 日期     : 2026-05-10
  * 版本     : v1.0
  * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
  ******************************************************************************
  */

/* 头文件说明：
 * pid.h     : 定义 PID_TypeDef 结构体及本文件对外函数原型。
 * encoder.h : 提供 ENCODER_SAMPLE_MS，作为 PID_Calc() 默认采样周期来源。
 * string.h  : 提供 memset()，用于 PID 结构体清零初始化。
 */
#include "pid.h"
#include "encoder.h"
#include <string.h>

/**
 * @brief  初始化 PID 控制器。
 * @param  pid：PID_TypeDef 指针，不能为空。
 * @param  kp：比例增益，单位取决于调用方输入/输出量纲。
 * @param  ki：积分增益，乘以误差积分后参与输出。
 * @param  kd：微分增益，乘以误差变化率后参与输出。
 * @param  out_max：输出上限。
 * @param  out_min：输出下限。
 * @retval 无。
 * @note   会清空积分项和上次误差；默认 integral_max = out_max。
 *         WARNING：如果 out_max 为负或小于 out_min，会导致限幅逻辑不符合预期。
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float out_min)
{
  if (pid == NULL) return;
  memset(pid, 0, sizeof(PID_TypeDef));
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->out_max = out_max;
  pid->out_min = out_min;
  pid->integral_max = out_max;  /* 默认积分限幅等于输出限幅 */
}

/**
 * @brief  执行一次 PID 计算。
 * @param  pid：PID_TypeDef 指针，不能为空。
 * @param  target：目标值，单位由调用方定义。
 * @param  current：当前反馈值，单位必须与 target 一致。
 * @retval float：PID 输出，已限制在 [out_min, out_max]。
 * @note   算法：
 *         error = target - current；
 *         P = Kp * error；
 *         I = Ki * integral(error * dt)，带 integral_max 限幅；
 *         D = Kd * (error - last_error) / dt。
 *         当前 dt 固定为 ENCODER_SAMPLE_MS/1000 秒。
 */
float PID_Calc(PID_TypeDef *pid, float target, float current)
{
  if (pid == NULL) return 0.0f;

  float error = target - current;
  float dt = (float)ENCODER_SAMPLE_MS / 1000.0f;  /* 采样周期(秒) */

  /* P */
  float p_out = pid->Kp * error;

  /* I */
  pid->integral += error * dt;
  if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
  if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
  float i_out = pid->Ki * pid->integral;

  /* D */
  float derivative = (error - pid->last_error) / dt;
  pid->last_error = error;
  float d_out = pid->Kd * derivative;

  float output = p_out + i_out + d_out;

  /* 输出限幅 */
  if (output > pid->out_max) output = pid->out_max;
  if (output < pid->out_min) output = pid->out_min;

  return output;
}

/**
 * @brief  复位 PID 的动态状态。
 * @param  pid：PID_TypeDef 指针，不能为空。
 * @retval 无。
 * @note   仅清除 integral 和 last_error，不改变 Kp/Ki/Kd 与输出限幅。
 *         适用于停车、模式切换或目标突变时防止积分残留。
 */
void PID_Reset(PID_TypeDef *pid)
{
  if (pid == NULL) return;
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
}

/**
 * @brief  设置积分项限幅。
 * @param  pid：PID_TypeDef 指针，不能为空。
 * @param  limit：积分限幅绝对值，建议为非负数。
 * @retval 无。
 * @note   PID_Calc() 内部将 integral 限制在 [-limit, +limit]。
 */
void PID_SetIntegralLimit(PID_TypeDef *pid, float limit)
{
  if (pid == NULL) return;
  pid->integral_max = limit;
}
