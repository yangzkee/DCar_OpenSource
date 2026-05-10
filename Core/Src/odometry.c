/**
 * @file    odometry.c
 * @brief   四轮全向底盘里程计估计算法实现。
 *
 * 本文件使用四个电机的轮端线速度反解车体坐标系速度，再结合 IMU yaw
 * 将车体速度投影到世界坐标系并积分得到 x/y 位置。
 *
 * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
 * 调用周期 : 建议 100 Hz，dt=0.01 s
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 */

/* 头文件说明：
 * odometry.h       : 定义 Odometry_TypeDef 和本模块对外接口。
 * motor_control.h  : 提供 MotorControl_GetSpeedMMps()，读取四轮实际线速度。
 * math.h           : 提供 cosf()、sinf()，用于坐标系旋转。
 */
#include "odometry.h"
#include "motor_control.h"
#include <math.h>

/* 里程计全局状态。
 * x/y  : 世界坐标系位置，单位 mm。
 * yaw  : 当前航向角，单位 degree，直接来自 IMU。
 * vx/vy: 车体坐标系速度，单位 mm/s。
 * w    : 旋转等效轮速分量，单位 mm/s。
 *
 * 初始值为 0：上电位置作为世界坐标原点。
 * 使用限制：该结构体非 volatile，当前假设只在主循环上下文读写。
 */
static Odometry_TypeDef odo = {0};

/**
 * @brief  初始化里程计。
 * @param  无。
 * @retval 无。
 * @note   当前实现直接调用 Odometry_Reset()，将当前位置作为坐标原点。
 */
void Odometry_Init(void) { Odometry_Reset(); }

/**
 * @brief  更新里程计 (建议在 100Hz 频率调用)
 * @param  imu_yaw：当前 IMU 提供的航向角，单位 degree。
 * @param  dt：两次更新之间的时间间隔，单位 s；100 Hz 时为 0.01f。
 * @retval 无。
 * @note   四轮速度反解公式与 Motion_Decouple() 互为逆运算：
 *         vy = (s1+s2+s3+s4)/4；
 *         vx = (s1-s2+s3-s4)/4；
 *         w  = (s1-s2-s3+s4)/4。
 *         WARNING：如果电机编号、安装方向或轮子位置发生变化，必须同步校正本公式。
 */
void Odometry_Update(float imu_yaw, float dt) {
  /* 1. 获取四个电机的实际线速度 (mm/s) */
  /* 这里 MotorControl_GetSpeedMMps 已经处理了电机极性和方向 */
  float s1 = MotorControl_GetSpeedMMps(0); /* 左前 */
  float s2 = MotorControl_GetSpeedMMps(1); /* 右前 */
  float s3 = MotorControl_GetSpeedMMps(2); /* 右后 */
  float s4 = MotorControl_GetSpeedMMps(3); /* 左后 */

  /* 2. 四轮编码器反解 Vx, Vy, w (机器人坐标系) */
  /* Vy: 前后 (+为前)
   * Vx: 左右 (+为右)
   * w:  旋转 (+为顺时针旋转，对应 Motion_Decouple 定义)
   * 注意：这些公式是 remote_to_motion.c 中解耦公式的逆运算
   */
  odo.vy = (s1 + s2 + s3 + s4) / 4.0f;
  odo.vx = (s1 - s2 + s3 - s4) / 4.0f;
  odo.w = (s1 - s2 - s3 + s4) / 4.0f;

  /* 3. 更新航向角 (直接取 IMU 值) */
  odo.yaw = imu_yaw;

  /* 4. 车体系到世界系变换并积分 (右手系定义：+Y 向前, +X 向右)
   * 当 Yaw = 0 时: dx = vx, dy = vy
   * 当 Yaw > 0 (顺时针旋转) 时: 车头向右偏，前进方向的分量会投影到 X 轴
   */
  float yaw_rad = odo.yaw * 0.01745329f; /* deg to rad */
  float cos_y = cosf(yaw_rad);
  float sin_y = sinf(yaw_rad);

  float dx = (odo.vx * cos_y + odo.vy * sin_y) * dt;
  float dy = (-odo.vx * sin_y + odo.vy * cos_y) * dt;

  odo.x += dx;
  odo.y += dy;
}

/**
 * @brief  获取当前里程计数据。
 * @param  data：输出结构体指针，不能为空；为空时不执行任何操作。
 * @retval 无。
 * @note   直接拷贝当前 odo 快照。若未来在中断中更新 odo，应在此处增加临界段。
 */
void Odometry_GetData(Odometry_TypeDef *data) {
  if (data) {
    *data = odo;
  }
}

/**
 * @brief  重置里程计坐标和速度。
 * @param  无。
 * @retval 无。
 * @note   将 x/y/yaw/vx/vy/w 全部清零；通常在上电初始化或重新设定原点时调用。
 */
void Odometry_Reset(void) {
  odo.x = 0;
  odo.y = 0;
  odo.yaw = 0;
  odo.vx = 0;
  odo.vy = 0;
  odo.w = 0;
}
