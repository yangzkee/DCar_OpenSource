/**
 ******************************************************************************
 * @file    remote_to_motion.c
 * @brief   遥控输入到四轮全向底盘运动控制的转换实现。
 *
 * 本文件负责把 PS2/SBUS 遥控器摇杆量转换为车体坐标系速度，并在用户
 * 没有给出旋转指令时使用 yaw PID 做“锁头”航向保持。最终输出的四个
 * 轮端目标线速度由 motor_control.c 的速度闭环执行。
 *
 * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX 生成 HAL 工程
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz（见 SystemClock_Config）
 * 相关外设 : USART2 接收 SBUS，TIM8 输出四路 PWM，TIM2/3/4/5 读取编码器
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 ******************************************************************************
 */

/* 头文件说明：
 * remote_to_motion.h : 本模块对外接口声明，以及遥控速度范围、死区等宏定义。
 * motor_control.h    : 提供 MotorControl_SetAllTargetSpeedMMps()，用于下发四轮速度。
 * pid.h              : 提供 PID_TypeDef、PID_Init()、PID_Calc()、PID_Reset()。
 * ps2_receiver.h     : 提供 PS2_Data_TypeDef 和 PS2_Receiver_GetData()。
 * math.h             : 提供 fabsf()，用于判断摇杆/旋转指令是否进入死区。
 */
#include "remote_to_motion.h"
#include "motor_control.h"
#include "pid.h"
#include "ps2_receiver.h"
#include <math.h>

/* Yaw 轴锁定相关变量。
 * yaw_pid       : 航向保持 PID，输入/反馈单位为 degree，输出单位为 mm/s 等效轮端旋转速度。
 * target_yaw    : 锁头目标航向角，单位 degree；首次进入锁头状态时取当前 IMU yaw。
 * is_yaw_locked : 锁头状态标志，0=未锁定，1=正在保持 target_yaw。
 *
 * 使用限制：
 * - 这些静态变量只在本文件访问；如果在中断中调用 Motion_HandleManual()，需要额外加临界段。
 * - 当前工程在 100 Hz 主循环中调用，未设计为多线程/RTOS 并发访问。
 */
static PID_TypeDef yaw_pid;
static float target_yaw = 0.0f;
static uint8_t is_yaw_locked = 0;

/* 手动速度指令死区，单位 mm/s。
 * 数值来源：用于过滤摇杆回中抖动和上位机小幅残余速度，低于 5 mm/s 认为无有效平移/旋转指令。
 */
#define MANUAL_CMD_DEADBAND_MMPS 5.0f

/* 航向保持误差死区，单位 degree。
 * yaw 偏差小于 0.3 度时不输出旋转补偿，避免电机在静止时因 IMU 噪声轻微啸叫。
 */
#define YAW_HOLD_DEADBAND_DEG 0.3f

/**
 * @brief  将 SBUS 摇杆原始值转换为车体速度，并应用中心死区。
 * @param  raw：SBUS 通道原始值，典型范围约 240~1807，中心值 REMOTE_CENTER。
 * @retval 速度，单位 mm/s；死区内返回 0，低端约映射到 -REMOTE_MAX_SPEED，
 *         高端约映射到 +REMOTE_MAX_SPEED。
 * @note   计算公式：
 *         speed = (raw - REMOTE_CENTER) * REMOTE_MAX_SPEED / REMOTE_RANGE。
 *         WARNING：REMOTE_CENTER/REMOTE_RANGE 应与实际接收机校准值一致，否则车辆会漂移。
 */
static inline float RawToSpeedWithDeadzone(uint16_t raw) {
  if (raw >= REMOTE_CENTER - REMOTE_DEADZONE &&
      raw <= REMOTE_CENTER + REMOTE_DEADZONE) {
    return 0.0f;
  }
  return ((float)((int32_t)raw - REMOTE_CENTER)) * REMOTE_MAX_SPEED /
         REMOTE_RANGE;
}

/**
 * @brief  将车体三自由度速度解耦为四个轮子的目标线速度。
 * @param  Vy：车体 Y 方向速度，单位 mm/s，正值表示向前。
 * @param  Vx：车体 X 方向速度，单位 mm/s，正值表示向右。
 * @param  w：车体旋转等效速度，单位 mm/s，正值表示顺时针旋转。
 * @retval 无。
 * @note   四轮全向/麦克纳姆式运动学：
 *         s1 = Vy + Vx + w，s2 = Vy - Vx - w，
 *         s3 = Vy + Vx - w，s4 = Vy - Vx + w。
 *         这里的 s1~s4 单位均为 mm/s，后续由电机速度环转换为编码器计数。
 */
static void Motion_Decouple(float Vy, float Vx, float w) {
  /* 电机物理位置: s1=左前(PC6), s2=右前(PC7), s3=右后(PC8), s4=左后(PC9) */
  float s1 = Vy + Vx + w; /* 左前 */
  float s2 = Vy - Vx - w; /* 右前 */
  float s3 = Vy + Vx - w; /* 右后 */
  float s4 = Vy - Vx + w; /* 左后 */

  MotorControl_SetAllTargetSpeedMMps(s1, s2, s3, s4);
}

/**
 * @brief  初始化遥控运动转换模块。
 * @param  无。
 * @retval 无。
 * @note   初始化 yaw 保持 PID，并将四个电机目标速度清零。
 *         本函数应在 MotorControl_Init() 之后调用，确保电机控制模块已可接收目标速度。
 */
void RemoteToMotion_Init(void) {
  /* Yaw PID: Kp=60, Ki=0.7, Kd=0.0, 输出限幅 ±800 mm/s */
  PID_Init(&yaw_pid, 60.0f, 0.7f, 0.0f, 800.0f, -800.0f);

  /* 初始静止 */
  MotorControl_SetAllTargetSpeedMMps(0.0f, 0.0f, 0.0f, 0.0f);
}

/**
 * @brief  从 PS2/SBUS 接收模块读取遥控器数据，并转换为运动控制命令。
 * @param  current_yaw：当前 IMU 航向角，单位 degree，用于锁头控制。
 * @param  is_remote_enabled：遥控使能标志，0=忽略遥控输入，非 0=允许遥控。
 * @retval 无。
 * @note   当遥控未使能或 SBUS 未连接时，vx/vy/w 均保持 0。
 *         实际锁头逻辑由 Motion_HandleManual() 完成。
 */
void RemoteToMotion_Update(float current_yaw, uint8_t is_remote_enabled) {
  PS2_Data_TypeDef data;
  PS2_Receiver_GetData(&data);

  float Vy = 0.0f;
  float Vx = 0.0f;
  float w = 0.0f;

  if (is_remote_enabled && data.connected) {
    Vy = RawToSpeedWithDeadzone(data.ly);
    Vx = RawToSpeedWithDeadzone(data.lx);
    w = RawToSpeedWithDeadzone(data.rx);
  }

  Motion_HandleManual(Vx, Vy, w, current_yaw);
}

/**
 * @brief  处理手动速度指令，并在无旋转指令时自动保持航向。
 * @param  vx：车体 X 方向速度，单位 mm/s，正值向右。
 * @param  vy：车体 Y 方向速度，单位 mm/s，正值向前。
 * @param  w：手动旋转速度，单位 mm/s 等效轮端速度，正值顺时针。
 * @param  current_yaw：当前 IMU 航向角，单位 degree。
 * @retval 无。
 * @note   状态机逻辑：
 *         1. 无平移且无旋转：退出锁头，目标 yaw 追随当前 yaw，四轮停止。
 *         2. 有旋转：认为用户主动转向，退出锁头并重置 PID。
 *         3. 有平移但无旋转：首次进入时锁定当前 yaw，之后用 PID 输出 w。
 *         WARNING：若 IMU yaw 方向与底盘旋转方向相反，需要调整 yaw PID 输出符号或 IMU yaw 符号。
 */
void Motion_HandleManual(float vx, float vy, float w, float current_yaw) {
  uint8_t has_translation = (fabsf(vx) > MANUAL_CMD_DEADBAND_MMPS) ||
                            (fabsf(vy) > MANUAL_CMD_DEADBAND_MMPS);
  uint8_t has_rotation = fabsf(w) > MANUAL_CMD_DEADBAND_MMPS;

  /* 锁头代码讲解按三个状态讲：
   * 1) 没有平移也没有旋转：退出锁头并停车；
   * 2) 有手动旋转：用户接管 yaw，重置锁头目标；
   * 3) 有平移但无旋转：保持 target_yaw，PID 输出纠偏 w。
   */
  if (!has_translation && !has_rotation) {
    is_yaw_locked = 0;
    target_yaw = current_yaw;
    PID_Reset(&yaw_pid);
    Motion_Decouple(0.0f, 0.0f, 0.0f);
    return;
  }
  /* 锁头逻辑实现 (核心：只要没有手动旋转指令 w，就自动维持当前 yaw) */
  if (has_rotation) {
    /* 用户手动主动转弯中，清除偏差状态 */
    is_yaw_locked = 0;
    target_yaw = current_yaw;
    PID_Reset(&yaw_pid);
  } else {
    /* 旋转指令为 0：进入/维持航向保持模式 */
    if (!is_yaw_locked) {
      is_yaw_locked = 1;
      /* 第一次进入锁头时，把当前朝向记录为要保持的 target_yaw。 */
      target_yaw = current_yaw;
      PID_Reset(&yaw_pid);
    }

    /* 死区：减小电机在极其细微误差下的啸叫 */
    float yaw_error = target_yaw - current_yaw;
    if (fabsf(yaw_error) < YAW_HOLD_DEADBAND_DEG) {
      w = 0.0f;
    } else {
      /* 航向闭环核心：target_yaw - current_yaw -> PID -> 纠偏旋转速度 w。 */
      w = PID_Calc(&yaw_pid, target_yaw, current_yaw);
    }
  }

  /* 最终解耦并下发到电机 */
  Motion_Decouple(vy, vx, w);
}
