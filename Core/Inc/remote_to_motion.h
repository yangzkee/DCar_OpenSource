/**
 ******************************************************************************
 * @file    remote_to_motion.h
 * @brief   遥控信号转车体目标速度
 *
 * 硬件平台 : STM32F407VETx，USART2 接收 SBUS，TIM8 输出四轮 PWM
 * 开发环境 : Keil uVision / MDK-ARM
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 *
 * 控制对应关系:
 *   LY  -> Vy  -> 前后移动 (正=前，负=后)
 *   LX  -> Vx  -> 左右移动 (正=右，负=左)
 *   RX  -> w   -> 旋转     (正=顺时针)
 *
 * 速度映射: SBUS 范围 240~1807，中位 1024 映射为 0，两端映射到
 *±REMOTE_MAX_SPEED mm/s
 ******************************************************************************
 */
#ifndef __REMOTE_TO_MOTION_H__
#define __REMOTE_TO_MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif

/* main.h：提供 HAL 基础类型和 uint8_t/uint16_t 间接依赖。 */
#include "main.h"

/* SBUS 输入范围。
 * 数值来源：当前接收机实测/常见 SBUS 摇杆有效范围，单位为通道原始计数。
 */
#define REMOTE_SBUS_MIN 240
#define REMOTE_SBUS_MAX 1807

/* SBUS 中位值，摇杆居中附近约为 1024。 */
#define REMOTE_CENTER 1024

/* 有效半程，用于速度缩放。
 * 公式：REMOTE_RANGE = REMOTE_CENTER - REMOTE_SBUS_MIN = 1024 - 240 = 784。
 */
#define REMOTE_RANGE (REMOTE_CENTER - REMOTE_SBUS_MIN) /* 784 */

/* 死区范围，单位为 SBUS 原始计数。
 * 摇杆在 [1024-200, 1024+200] 内视为居中，防止机械回中抖动造成慢速漂移。
 */
#define REMOTE_DEADZONE 200

/* 速度最大值，单位 mm/s，摇杆推满时 Vy/Vx/w 的幅值。 */
#define REMOTE_MAX_SPEED 800.0f

/**
 * @brief  遥控转运动模块初始化。
 * @param  无。
 * @retval 无。
 */
void RemoteToMotion_Init(void);

/**
 * @brief  根据遥控信号更新车体目标速度。
 * @param  current_yaw：当前航向角，单位 degree。
 * @param  is_remote_enabled：遥控使能标志，0=禁用，非 0=启用。
 * @retval 无。
 * @note   需在固定周期调用 (如 10 ms)，内部会设置四电机目标线速度 (mm/s)。
 */
void RemoteToMotion_Update(float current_yaw, uint8_t is_remote_enabled);

/**
 * @brief  处理手动运动指令 (含锁头逻辑)
 * @param  vx：车体 X 方向速度，单位 mm/s，正=右。
 * @param  vy：车体 Y 方向速度，单位 mm/s，正=前。
 * @param  w：旋转速度，单位 mm/s，非 0 时会更新锁头目标。
 * @param  current_yaw：当前航向角，单位 degree。
 * @retval 无。
 */
void Motion_HandleManual(float vx, float vy, float w, float current_yaw);

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_TO_MOTION_H__ */
