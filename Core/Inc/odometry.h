/**
 * @file    odometry.h
 * @brief   里程计估计算法接口声明
 *          利用四轮编码器及 IMU Yaw 轴进行航位推算
 *
 * 硬件平台 : STM32F407VETx，TIM2/3/4/5 编码器 + ICM20602 yaw
 * 开发环境 : Keil uVision / MDK-ARM
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 */
#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* main.h：提供 HAL 工程基础类型。 */
#include "main.h"

/**
 * @brief  里程计数据结构
 * @note   世界坐标系单位为 mm 和 degree；车体速度单位为 mm/s。
 */
typedef struct {
  float x;   /* 世界坐标系 X (mm) */
  float y;   /* 世界坐标系 Y (mm) */
  float yaw; /* 世界坐标系航向角 (degree) */

  float vx; /* 机器人体系 X 速度 (mm/s) */
  float vy; /* 机器人体系 Y 速度 (mm/s) */
  float w;  /* 机器人体系旋转角速度 (计算值) */
} Odometry_TypeDef;

/**
 * @brief  初始化里程计。
 * @param  无。
 * @retval 无。
 */
void Odometry_Init(void);

/**
 * @brief  更新里程计 (建议在 100Hz 频率调用)
 * @param  imu_yaw：当前 IMU 提供的航向角，单位 degree。
 * @param  dt：两次更新之间的时间间隔，单位 s。
 * @retval 无。
 */
void Odometry_Update(float imu_yaw, float dt);

/**
 * @brief  获取当前里程计数据。
 * @param  data：输出结构体指针。
 * @retval 无。
 */
void Odometry_GetData(Odometry_TypeDef *data);

/**
 * @brief  重置里程计坐标。
 * @param  无。
 * @retval 无。
 */
void Odometry_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __ODOMETRY_H__ */
