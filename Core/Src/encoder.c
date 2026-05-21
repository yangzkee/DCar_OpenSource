/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   四路电机编码器读取与速度估算实现。
  *
  * 本文件使用 STM32F407 的 TIM2/TIM3/TIM4/TIM5 编码器接口模式读取四个
  * 电机的 AB 相正交编码器计数，并在固定 10 ms 周期内计算编码器计数速度
  * 和轮端线速度。
  *
  * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
  * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * 外设映射 : M1=TIM2(PA5/PB3)，M2=TIM3(PA6/PA7)，
  *            M3=TIM4(PB6/PB7)，M4=TIM5(PA0/PA1)
  * 作者     : DCar_OpenSource
  * 日期     : 2026-05-10
  * 版本     : v1.0
  * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
  ******************************************************************************
  */

/* 头文件说明：
 * encoder.h : 本模块对外接口、MOTOR_NUM/ENC_Mx、编码器线数、减速比、
 *             轮径、COUNTS_PER_MM 等换算宏定义。
 * tim.h     : 提供 htim2/htim3/htim4/htim5 定时器句柄和 HAL TIM 宏接口。
 * string.h  : 提供 memset()，用于初始化静态数组。
 */
#include "encoder.h"
#include "tim.h"
#include <string.h>

/* 编码器累计计数值，单位 count。
 * encoder_count      : 软件累计计数，int32_t 可跨越 16 位定时器回绕。
 * encoder_count_last : 上一次采样的硬件 CNT 值，低 16 位有效。
 * encoder_speed      : 当前速度，单位 count/s，已乘以 DF_WHEEL_POLARITY。
 *
 * 初始值为 0：上电后以当前位置作为计数零点。
 * 使用限制：Encoder_UpdateSpeed() 应固定周期调用；若周期变化，速度换算会失真。
 */
static int32_t encoder_count[MOTOR_NUM];
static int32_t encoder_count_last[MOTOR_NUM];
static float encoder_speed[MOTOR_NUM];

/**
 * @brief  将两个 16 位定时器计数值换算为有符号增量。
 * @param  now：当前 TIMx->CNT 低 16 位值，范围 0~65535。
 * @param  last：上次采样 TIMx->CNT 低 16 位值，范围 0~65535。
 * @retval int16_t：本周期计数增量，范围 -32768~32767。
 * @note   通过 C 语言无符号减法自然回绕，再强制转换为 int16_t：
 *         例如 now=0x0002、last=0xFFFE 时，now-last=0x0004，表示正向跨零。
 *         WARNING：单个采样周期内计数变化量不能超过 int16_t 可表达范围，
 *         否则无法区分高速运动和反向回绕。
 */
static inline int16_t count_to_delta(uint16_t now, uint16_t last)
{
  return (int16_t)(now - last);
}

/**
 * @brief  初始化四路编码器模块并启动 TIM 编码器接口。
 * @param  无。
 * @retval 无。
 * @note   会清零软件累计计数和速度数组，然后启动 TIM2/3/4/5 的
 *         TIM_CHANNEL_ALL。调用前应已执行 MX_TIM2/3/4/5_Init()。
 */
void Encoder_Init(void)
{
  memset(encoder_count, 0, sizeof(encoder_count));
  memset(encoder_count_last, 0, sizeof(encoder_count_last));
  memset(encoder_speed, 0, sizeof(encoder_speed));

  /* 编码器代码讲解入口：
   * 这四行只负责“开始数 AB 相脉冲”，真正的测速在 Encoder_UpdateSpeed()。
   * 录课时按 TIM2/TIM3/TIM4/TIM5 -> 四个轮子的顺序讲清硬件映射。
   */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

/**
 * @brief  读取指定电机对应定时器的原始 CNT 寄存器值。
 * @param  motor_id：电机编号，取值 ENC_M1~ENC_M4，即 0~3。
 * @retval uint16_t：TIMx->CNT 原始值，范围 0~65535；motor_id 非法时返回 0。
 * @note   该函数只读取硬件计数器，不更新 encoder_speed。
 */
uint16_t Encoder_GetCount(uint8_t motor_id)
{
  if (motor_id >= MOTOR_NUM) return 0;

  switch (motor_id) {
    case ENC_M1: return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    case ENC_M2: return (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    case ENC_M3: return (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    case ENC_M4: return (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);
    default: return 0;
  }
}

/**
 * @brief  按固定采样周期更新四个电机的编码器速度。
 * @param  无。
 * @retval 无。
 * @note   调用周期由 ENCODER_SAMPLE_MS 指定，当前为 10 ms。
 *         速度公式：
 *         encoder_speed = delta_count * 1000 / ENCODER_SAMPLE_MS，单位 count/s。
 *         WARNING：本函数通常在 TIM6 周期中断链路中被 MotorControl_Update()
 *         调用，执行时间应保持短小，避免阻塞其他中断。
 */
void Encoder_UpdateSpeed(void)
{
  uint16_t raw[MOTOR_NUM];
  /* 先在同一个 10ms 控制拍里读四个 CNT 快照。
   * 讲解重点：TIMx 编码器模式已经在硬件里自动完成正反向计数，
   * 软件这里只需要读取当前计数值，不需要手动判断 A/B 相先后。
   */
  raw[ENC_M1] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  raw[ENC_M2] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  raw[ENC_M3] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  raw[ENC_M4] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);

  for (uint8_t i = 0; i < MOTOR_NUM; i++) {
    /* 录编码器代码讲解时按这条链路讲：
     * raw CNT -> delta_count -> count/s -> mm/s。
     * count_to_delta() 专门处理 16 位计数器跨 0 回绕的问题。
     */
    int16_t delta = count_to_delta(raw[i], (uint16_t)encoder_count_last[i]);
    encoder_count[i] += delta;
    encoder_count_last[i] = raw[i];
    /* 速度 = 增量 / 时间(s)，应用编码器极性 */
    encoder_speed[i] = (float)(delta * DF_WHEEL_POLARITY) * 1000.0f / (float)ENCODER_SAMPLE_MS;
  }
}

/**
 * @brief  获取指定电机最近一次计算得到的编码器速度。
 * @param  motor_id：电机编号，取值 0~3。
 * @retval float：速度，单位 count/s；非法编号返回 0.0f。
 * @note   返回值由 Encoder_UpdateSpeed() 刷新，未刷新前保持上次结果。
 */
float Encoder_GetSpeed(uint8_t motor_id)
{
  if (motor_id >= MOTOR_NUM) return 0.0f;
  return encoder_speed[motor_id];
}

/**
 * @brief  获取指定电机轮端线速度。
 * @param  motor_id：电机编号，取值 0~3。
 * @retval float：轮端线速度，单位 mm/s；非法编号返回 0.0f。
 * @note   换算公式：mm/s = count/s / COUNTS_PER_MM。
 *         COUNTS_PER_MM 来自编码器线数、4 倍频、减速比和轮径计算。
 */
float Encoder_GetSpeedMMps(uint8_t motor_id)
{
  if (motor_id >= MOTOR_NUM) return 0.0f;
  /* 计数/秒 -> mm/s: 除以每 mm 的计数 */
  return encoder_speed[motor_id] / COUNTS_PER_MM;
}

/**
 * @brief  清除指定电机的软件累计计数、速度和硬件 CNT 寄存器。
 * @param  motor_id：电机编号，取值 0~3。
 * @retval 无。
 * @note   会把对应 TIMx->CNT 写 0。WARNING：如果电机正在转动，清零瞬间
 *         可能造成下一次速度估算异常，建议在停车状态调用。
 */
void Encoder_ClearCount(uint8_t motor_id)
{
  if (motor_id >= MOTOR_NUM) return;
  encoder_count[motor_id] = 0;
  encoder_count_last[motor_id] = 0;
  encoder_speed[motor_id] = 0.0f;
  switch (motor_id) {
    case ENC_M1: __HAL_TIM_SET_COUNTER(&htim2, 0); break;
    case ENC_M2: __HAL_TIM_SET_COUNTER(&htim3, 0); break;
    case ENC_M3: __HAL_TIM_SET_COUNTER(&htim4, 0); break;
    case ENC_M4: __HAL_TIM_SET_COUNTER(&htim5, 0); break;
    default: break;
  }
}

/**
 * @brief  清除四个电机的编码器计数和速度。
 * @param  无。
 * @retval 无。
 * @note   依次调用 Encoder_ClearCount()，用于上电初始化或重新设定零点。
 */
void Encoder_ClearAll(void)
{
  for (uint8_t i = 0; i < MOTOR_NUM; i++) {
    Encoder_ClearCount(i);
  }
}
