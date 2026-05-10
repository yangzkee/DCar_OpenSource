/**
 * @file    move_control.h
 * @brief   位置环闭环控制模块 (相对位移)
 *
 * 硬件平台 : STM32F407VETx，四轮全向/麦轮底盘
 * 开发环境 : Keil uVision / MDK-ARM
 * 晶振频率 : HSE 8 MHz，SYSCLK 168 MHz
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 */
#ifndef __MOVE_CONTROL_H__
#define __MOVE_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* main.h：提供基础类型定义和 HAL 工程公共声明。 */
#include "main.h"

/**
 * @brief  移动任务状态
 * @note   状态机流程：
 *         MOVE_IDLE      ：无位移任务，电机由其他控制源接管或保持停止；
 *         MOVE_EXECUTING ：已设置相对位移目标，周期更新位置误差并输出速度；
 *         MOVE_FINISHED  ：任务达到停止条件，等待上层读取或设置新目标。
 */
typedef enum {
  MOVE_IDLE,      /* 空闲 */
  MOVE_EXECUTING, /* 执行中 */
  MOVE_FINISHED   /* 已完成 */
} MoveState_t;

/**
 * @brief  初始化移动控制模块
 * @param  无。
 * @retval 无。
 * @note   清零内部目标、误差积分/状态变量，并进入 MOVE_IDLE。
 *         调用顺序建议在 Odometry_Init() 和 MotorControl_Init() 之后。
 */
void MoveControl_Init(void);

/**
 * @brief  设置相对位移目标
 * @param  rel_x: 相对 X 位移 (mm, +为右)
 * @param  rel_y: 相对 Y 位移 (mm, +为前)
 * @param  target_yaw: 目标航向角 (degree)，通常传当前角度以维持直行
 * @param  speed_limit_mmps：速度上限，单位 mm/s，用于限制位置环输出幅值。
 * @retval 无。
 * @note   坐标系为车体/里程计约定坐标系；设置目标后状态切换为 MOVE_EXECUTING。
 *         WARNING：目标位移过大或速度上限过高时，需确认场地空间和电机供电裕量。
 */
void MoveControl_SetRelativeTarget(float rel_x, float rel_y, float target_yaw,
                                   float speed_limit_mmps);

/**
 * @brief  主更新循环 (建议 100Hz 频率调用)
 * @param  无。
 * @retval 无。
 * @note   状态机逻辑：
 *         1. MOVE_IDLE：不输出新的位移控制速度；
 *         2. MOVE_EXECUTING：读取里程计位置，计算目标误差和限幅速度；
 *         3. 满足到位阈值后切换 MOVE_FINISHED 并停止位移输出。
 *         应与里程计更新保持相近周期，避免位置反馈滞后。
 */
void MoveControl_Update(void);

/**
 * @brief  获取当前移动状态
 * @param  无。
 * @retval MoveState_t：MOVE_IDLE/MOVE_EXECUTING/MOVE_FINISHED。
 * @note   上层可根据该状态决定是否发送下一段位移指令。
 */
MoveState_t MoveControl_GetState(void);

/**
 * @brief  紧急停止移动任务
 * @param  无。
 * @retval 无。
 * @note   立即清除当前位移任务并切换到空闲/停止状态。
 *         WARNING：该接口仅停止位移任务，若其他控制源仍在给电机目标速度，
 *         需要上层同时处理控制源互斥。
 */
void MoveControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOVE_CONTROL_H__ */
