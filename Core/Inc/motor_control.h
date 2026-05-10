/**
 ******************************************************************************
 * @file    motor_control.h
 * @brief   四电机 PID 速度闭环控制模块
 *          整合编码器 + PID + PWM 输出
 *
 * 硬件平台 : STM32F407VETx，TIM8 PWM + GPIOD 方向控制
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
 * 晶振频率 : HSE 8 MHz，SYSCLK 168 MHz
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 ******************************************************************************
 */
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* encoder.h：提供编码器速度、线速度、COUNTS_PER_MM 等反馈和换算接口。 */
#include "encoder.h"
/* main.h：提供 GPIO 引脚宏、HAL 类型和基础整数类型。 */
#include "main.h"

/* PWM 周期，单位为 TIM8 计数。
 * 数值来源：tim.c 中 TIM8.Init.Period = 999。
 * 占空比公式：duty = CCRx / (MOTOR_PWM_PERIOD + 1)，因此 999 约等于 99.9%。
 * WARNING：如果 CubeMX 中修改 TIM8 Period，此处必须同步修改。
 */
#define MOTOR_PWM_PERIOD 999

/* 默认 PID 参数。
 * Kp：比例项，提升速度误差响应；
 * Ki：积分项，用于克服静摩擦和负载偏差；
 * Kd：微分项，当前为 0，避免编码器离散计数噪声放大。
 * 参数量纲与 MotorControl_Update() 内部的“每 10 ms 编码器增量”一致。
 */
#define MOTOR_PID_KP_DEFAULT 3.0f
#define MOTOR_PID_KI_DEFAULT 1.5f
#define MOTOR_PID_KD_DEFAULT 0.0f

/**
 * @brief  电机控制模块初始化
 * @param  无。
 * @retval 无。
 * @note   会初始化编码器、启动 TIM6 中断、启动 TIM8 四路 PWM，并设置默认 PID 参数。
 *         调用前必须已完成 MX_GPIO_Init()、MX_TIM2/3/4/5/6/8_Init()。
 */
void MotorControl_Init(void);

/**
 * @brief  设置指定电机的目标速度 (编码器计数/秒)
 * @param  motor_id: 电机编号 0~3
 * @param  target_speed: 目标速度，正为正向，负为反向
 * @retval 无。
 * @note   该接口直接使用 count/s，适合底层调试；上层运动控制建议使用 mm/s 接口。
 */
void MotorControl_SetTargetSpeed(uint8_t motor_id, float target_speed);

/**
 * @brief  设置四个电机的目标速度 (编码器计数/秒)
 * @param  s1：M1 目标速度，单位 count/s。
 * @param  s2：M2 目标速度，单位 count/s。
 * @param  s3：M3 目标速度，单位 count/s。
 * @param  s4：M4 目标速度，单位 count/s。
 * @retval 无。
 * @note   正负号按本工程统一运动学方向解释，实际电机安装方向在 .c 内部修正。
 */
void MotorControl_SetAllTargetSpeed(float s1, float s2, float s3, float s4);

/**
 * @brief  设置四个电机的目标线速度 (mm/s)
 * @param  mm_s1：M1 目标轮端线速度，单位 mm/s。
 * @param  mm_s2：M2 目标轮端线速度，单位 mm/s。
 * @param  mm_s3：M3 目标轮端线速度，单位 mm/s。
 * @param  mm_s4：M4 目标轮端线速度，单位 mm/s。
 * @retval 无。
 * @note   内部通过 COUNTS_PER_MM 换算为编码器 count/s。
 */
void MotorControl_SetAllTargetSpeedMMps(float mm_s1, float mm_s2, float mm_s3,
                                        float mm_s4);

/**
 * @brief  PID 速度环更新，需在固定周期调用 (如 10ms 定时器中断)
 * @param  无。
 * @retval 无。
 * @note   内部会调用 Encoder_UpdateSpeed，再对每个电机做 PID 并输出 PWM
 *         WARNING：该函数属于实时控制路径，不应加入 printf、HAL_Delay 等阻塞操作。
 */
void MotorControl_Update(void);

/**
 * @brief  设置指定电机的 PID 参数
 * @param  motor_id：电机编号 0~3。
 * @param  kp：比例增益。
 * @param  ki：积分增益。
 * @param  kd：微分增益。
 * @retval 无。
 * @note   设置后会重新初始化该电机 PID 状态，积分项和上次误差被清零。
 */
void MotorControl_SetPID(uint8_t motor_id, float kp, float ki, float kd);

/**
 * @brief  停止所有电机
 * @param  无。
 * @retval 无。
 * @note   清零四路目标速度、复位 PID、PWM 输出归零并让方向脚进入空载状态。
 *         WARNING：空载停止不是主动刹车，底盘可能因惯性继续移动。
 */
void MotorControl_StopAll(void);

/**
 * @brief  获取指定电机当前速度 (编码器计数/秒)
 * @param  motor_id：电机编号 0~3。
 * @retval 当前速度，单位 count/s；非法编号返回 0。
 * @note   返回值已按电机安装方向修正，可直接用于运动学计算。
 */
float MotorControl_GetSpeed(uint8_t motor_id);

/**
 * @brief  获取指定电机当前线速度 (mm/s)
 * @param  motor_id：电机编号 0~3。
 * @retval 当前轮端线速度，单位 mm/s；非法编号返回 0。
 * @note   返回值已按电机安装方向修正，可直接用于里程计计算。
 */
float MotorControl_GetSpeedMMps(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
