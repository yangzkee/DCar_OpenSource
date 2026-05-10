/**
 * @file    move_control.c
 * @brief   相对位移位置环闭环控制逻辑实现。
 *
 * 本文件根据上位机下发的相对位移目标，结合 odometry.c 输出的世界坐标
 * 位置和 IMU yaw，生成匀速平移速度与航向保持旋转速度，再解耦为四轮
 * 目标速度。
 *
 * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
 * 调用周期 : 建议 100 Hz，由 loop.c 的 LOOP_100HZ() 调用
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 */

/* 头文件说明：
 * move_control.h  : 本模块状态枚举 MoveState_t 和对外 API 声明。
 * motor_control.h : 提供 MotorControl_SetAllTargetSpeedMMps()，用于下发四轮目标速度。
 * odometry.h      : 提供 Odometry_GetData()，读取当前世界坐标位置与航向角。
 * pid.h           : 提供 PID 控制器结构体和初始化/计算/复位接口。
 * math.h          : 提供 sinf()、cosf()、sqrtf()，用于坐标变换和距离计算。
 */
#include "move_control.h"
#include "motor_control.h"
#include "odometry.h"
#include "pid.h"
#include <math.h>

/* 定制 PID 参数。
 * POS_KP/POS_KI/POS_KD：位置环 PID 参数，输入为位置误差 mm，输出为速度 mm/s。
 * YAW_KP              ：航向保持比例增益，输入为角度误差 degree，输出为旋转等效速度 mm/s。
 * WARNING：这些参数与底盘惯量、轮胎摩擦、电机响应有关，移植硬件后应重新整定。
 */
#define POS_KP 10.0f
#define POS_KI 0.0f
#define POS_KD 0.1f
#define YAW_KP 10.0f /* 航向保持 KP */

/* 运动限制常量。
 * MAX_SPEED        : 平移速度最大值，单位 mm/s。
 * MAX_W            : 旋转输出最大值，单位 mm/s，对应四轮速度解耦中的 w 分量。
 * ARRIVE_DEADZONE  : 到达判定半径，单位 mm；小于该距离认为目标完成。
 */
#define MAX_SPEED 500.0f      /* 最大平移速度 mm/s */
#define MAX_W 100.0f          /* 最大旋转速度 mm/s (对应轮端) */
#define ARRIVE_DEADZONE 10.0f /* 到达死区 10mm */

/* 模块静态状态。
 * current_state : 位置控制状态机，取值见 MoveState_t。
 * target_xw/yw  : 世界坐标系目标点，单位 mm。
 * target_yaw    : 目标航向角，单位 degree。
 * speed_limit   : 当前任务速度上限，单位 mm/s，默认 MAX_SPEED。
 * pid_x/y/w     : X/Y 位置环和 yaw 航向环 PID。
 */
static MoveState_t current_state = MOVE_IDLE;
static float target_xw = 0, target_yw = 0, target_yaw = 0;
static float speed_limit = MAX_SPEED; // 默认使用最大速度
static PID_TypeDef pid_x, pid_y, pid_w;

/**
 * @brief  初始化相对位移控制模块。
 * @param  无。
 * @retval 无。
 * @note   配置 X/Y 位置环和 yaw 航向环 PID 参数，并将状态机置为空闲。
 *         本函数应在 PID 模块可用后、进入主循环前调用。
 */
void MoveControl_Init(void) {
  /* 初始化位置 PID: 输入误差(mm) -> 输出速度(mm/s) */
  PID_Init(&pid_x, POS_KP, POS_KI, POS_KD, MAX_SPEED, -MAX_SPEED);
  PID_Init(&pid_y, POS_KP, POS_KI, POS_KD, MAX_SPEED, -MAX_SPEED);
  /* 初始化航向 PID: 输入误差(deg) -> 输出旋转分量(mm/s) */
  PID_Init(&pid_w, YAW_KP, 0, 0, MAX_W, -MAX_W);

  current_state = MOVE_IDLE;
}

/**
 * @brief  设置一个车体系相对位移目标。
 * @param  rel_x：相对 X 位移，单位 mm，正值表示车体右侧。
 * @param  rel_y：相对 Y 位移，单位 mm，正值表示车体前方。
 * @param  target_yaw_deg：目标航向角，单位 degree；通常传入当前 yaw 表示移动中保持朝向。
 * @param  speed_limit_mmps：本次任务速度上限，单位 mm/s；小于等于 0 时使用 MAX_SPEED。
 * @retval 无。
 * @note   函数内部把车体系相对位移旋转到世界坐标系：
 *         rel_xw = rel_x*cos(yaw) + rel_y*sin(yaw)
 *         rel_yw = -rel_x*sin(yaw) + rel_y*cos(yaw)
 *         然后用当前里程计位置加上相对量得到绝对目标点。
 */
void MoveControl_SetRelativeTarget(float rel_x, float rel_y,
                                   float target_yaw_deg,
                                   float speed_limit_mmps) {
  Odometry_TypeDef odo;
  Odometry_GetData(&odo);

  float yaw_rad = odo.yaw * 0.01745329f;
  float cos_y = cosf(yaw_rad);
  float sin_y = sinf(yaw_rad);
  float rel_xw = rel_x * cos_y + rel_y * sin_y;
  float rel_yw = -rel_x * sin_y + rel_y * cos_y;

  /* Convert body-relative displacement to a world-frame target. */
  target_xw = odo.x + rel_xw;
  target_yw = odo.y + rel_yw;
  target_yaw = target_yaw_deg;

  /* 设置速度上限 */
  if (speed_limit_mmps > 0) {
    speed_limit = (speed_limit_mmps > MAX_SPEED) ? MAX_SPEED : speed_limit_mmps;
  } else {
    speed_limit = MAX_SPEED;
  }

  /* 重置 PID 并更新速度限制 */
  PID_Init(&pid_x, POS_KP, POS_KI, POS_KD, speed_limit, -speed_limit);
  PID_Init(&pid_y, POS_KP, POS_KI, POS_KD, speed_limit, -speed_limit);
  PID_Reset(&pid_x);
  PID_Reset(&pid_y);
  PID_Reset(&pid_w);

  current_state = MOVE_EXECUTING;
}

/**
 * @brief  将车体速度解耦为四轮目标线速度。
 * @param  Vy：车体 Y 方向速度，单位 mm/s，正值向前。
 * @param  Vx：车体 X 方向速度，单位 mm/s，正值向右。
 * @param  w：旋转等效速度，单位 mm/s，正值顺时针。
 * @retval 无。
 * @note   与 remote_to_motion.c 的解耦公式保持一致，保证手动控制和位置控制的轮速符号统一。
 */
static void Motion_Decouple(float Vy, float Vx, float w) {
  /* 电机物理位置: s1=左前, s2=右前, s3=右后, s4=左后 */
  float s1 = Vy + Vx + w;
  float s2 = Vy - Vx - w;
  float s3 = Vy + Vx - w;
  float s4 = Vy - Vx + w;

  MotorControl_SetAllTargetSpeedMMps(s1, s2, s3, s4);
}

/**
 * @brief  位置控制主更新函数。
 * @param  无。
 * @retval 无。
 * @note   建议 100 Hz 调用。流程：
 *         1. 读取里程计，计算世界系目标误差。
 *         2. 距离小于 ARRIVE_DEADZONE 时停止并切换到 MOVE_FINISHED。
 *         3. 按 speed_limit 生成指向目标点的世界系匀速速度。
 *         4. 将世界系速度逆变换到车体系并下发四轮速度。
 *         WARNING：dt 不在本函数内显式使用，速度大小由 speed_limit 直接决定。
 */
void MoveControl_Update(void) {
  if (current_state != MOVE_EXECUTING)
    return;

  Odometry_TypeDef odo;
  Odometry_GetData(&odo);

  /* 1. 计算世界系误差 */
  float err_xw = target_xw - odo.x;
  float err_yw = target_yw - odo.y;

  /* 2. 检查是否到达 */
  float dist = sqrtf(err_xw * err_xw + err_yw * err_yw);
  if (dist < ARRIVE_DEADZONE) {
    current_state = MOVE_FINISHED;
    Motion_Decouple(0, 0, 0);
    return;
  }

  /* 3. Uniform-speed world-frame velocity toward the target. */
  float v_xw = err_xw / dist * speed_limit;
  float v_yw = err_yw / dist * speed_limit;
  float w_out = PID_Calc(&pid_w, target_yaw, odo.yaw);

  /* 4. 将世界系速度投影到机器人坐标系 */
  float yaw_rad = odo.yaw * 0.01745329f;
  float cos_y = cosf(yaw_rad);
  float sin_y = sinf(yaw_rad);

  /* 旋转矩阵逆变换: x_r = x_w*cos - y_w*sin, y_r = x_w*sin + y_w*cos */
  float vx_r = v_xw * cos_y - v_yw * sin_y;
  float vy_r = v_xw * sin_y + v_yw * cos_y;

  /* 5. 下发电机控制 */
  Motion_Decouple(vy_r, vx_r, w_out);
}

/**
 * @brief  获取当前位移控制状态。
 * @param  无。
 * @retval MoveState_t：MOVE_IDLE/MOVE_EXECUTING/MOVE_FINISHED。
 * @note   上层可通过该状态决定是否允许遥控手动速度覆盖。
 */
MoveState_t MoveControl_GetState(void) { return current_state; }

/**
 * @brief  停止当前位移控制任务。
 * @param  无。
 * @retval 无。
 * @note   将状态机置为空闲，并立即下发四轮 0 速度。
 */
void MoveControl_Stop(void) {
  current_state = MOVE_IDLE;
  Motion_Decouple(0, 0, 0);
}
