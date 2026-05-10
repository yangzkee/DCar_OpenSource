/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   四路电机编码器读取模块
 *          定时器编码器模式：TIM2(M1), TIM3(M2), TIM4(M3), TIM5(M4)
 *
 * 硬件平台 : STM32F407VETx，ARM Cortex-M4F
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
 * 晶振频率 : HSE 8 MHz，SYSCLK 168 MHz
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 ******************************************************************************
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* main.h：提供 HAL 基础类型、uint8_t/uint16_t/uint32_t 类型和定时器 GPIO 宏。 */
#include "main.h"

/* 电机数量。
 * 数值来源：底盘为四轮结构，M1~M4 共 4 路独立编码器。
 */
#define MOTOR_NUM 4

/* 编码器与电机对应关系。
 * 下标用于 encoder_count[]、encoder_speed[] 等数组索引，必须保持 0~3 连续。
 */
#define ENC_M1 0 /* TIM2 - PA5, PB3 */
#define ENC_M2 1 /* TIM3 - PA6, PA7 */
#define ENC_M3 2 /* TIM4 - PB6, PB7 */
#define ENC_M4 3 /* TIM5 - PA0, PA1 */

/* DF 电机参数。
 * DF_ENCODER_LINES    : 编码器线数，单位 pulse/rev，指电机轴单圈 A/B 相原始线数。
 * DF_MOTOR_REDUCTION  : 减速箱减速比，轮子转 1 圈时电机轴约转 38 圈。
 * DF_WHEEL_DIAMETER_MM: 轮子直径，单位 mm，用于线速度和里程换算。
 * DF_WHEEL_POLARITY   : 编码器极性修正，1=保持原方向，-1=整体取反。
 */
#define DF_ENCODER_LINES 448    /* 编码器线数（每圈脉冲数） */
#define DF_MOTOR_REDUCTION 38   /* 电机减速比 */
#define DF_WHEEL_DIAMETER_MM 97 /* 轮子直径 (mm) */
#define DF_WHEEL_POLARITY 1     /* 编码器极性（1 正 / -1 反） */

/* 编码器参数（由 DF 参数派生）。 */
#define ENCODER_PPR DF_ENCODER_LINES
/* 4 倍频：A/B 相上升沿和下降沿均计数，因此每线产生 4 个计数。 */
#define ENCODER_MULTIPLE 4 /* 4倍频 (AB相正交) */
/* 电机轴每转计数。
 * 公式：ENCODER_COUNTS_REV = 448 * 4 = 1792 count/rev。
 */
#define ENCODER_COUNTS_REV                                                     \
  (ENCODER_PPR * ENCODER_MULTIPLE) /* 电机轴每圈总计数 */

/* 轮子相关：轮子转一圈的编码器计数 = 电机轴计数 * 减速比。
 * 公式：WHEEL_COUNTS_REV = 1792 * 38 = 68096 count/rev。
 */
#define WHEEL_COUNTS_REV (ENCODER_COUNTS_REV * DF_MOTOR_REDUCTION)
/* 轮周长，单位 mm。
 * 公式：C = pi * D = 3.14159265 * 97 mm。
 */
#define WHEEL_CIRCUMFERENCE_MM (3.14159265f * (float)DF_WHEEL_DIAMETER_MM)
/* 每 mm 对应的编码器计数，单位 count/mm。
 * 公式：COUNTS_PER_MM = WHEEL_COUNTS_REV / WHEEL_CIRCUMFERENCE_MM。
 */
#define COUNTS_PER_MM ((float)WHEEL_COUNTS_REV / WHEEL_CIRCUMFERENCE_MM)

/* 速度计算周期，单位 ms。
 * 当前为 10 ms，需与 MotorControl_Update() 的调用周期一致；如果 TIM6 软件分频
 * 改变，这里必须同步修改，否则 count/s 和 mm/s 会计算错误。
 */
#define ENCODER_SAMPLE_MS 10

/**
 * @brief  编码器模块初始化，启动编码器
 * @param  无。
 * @retval 无。
 * @note   清零软件计数缓存，并启动 TIM2/TIM3/TIM4/TIM5 编码器接口。
 *         调用前需完成 MX_TIM2_Init()~MX_TIM5_Init()。
 */
void Encoder_Init(void);

/**
 * @brief  获取指定电机的编码器原始计数值
 * @param  motor_id: 电机编号 0~3 (M1~M4)
 * @retval 定时器 CNT 寄存器原始值，范围 0 ~ 65535
 * @note   直接读取硬件计数器，不做速度更新；用于调试或清零前检查。
 */
uint16_t Encoder_GetCount(uint8_t motor_id);

/**
 * @brief  获取指定电机的速度 (编码器计数/秒，已应用极性)
 * @param  motor_id: 电机编号 0~3
 * @retval 速度值，正为正向，负为反向
 */
float Encoder_GetSpeed(uint8_t motor_id);

/**
 * @brief  获取轮子线速度 (mm/s)
 * @param  motor_id: 电机编号 0~3
 * @retval 轮端线速度，单位 mm/s；非法编号返回 0。
 * @note   换算公式：speed_mmps = speed_count_per_s / COUNTS_PER_MM。
 */
float Encoder_GetSpeedMMps(uint8_t motor_id);

/**
 * @brief  速度计算更新，需在固定周期调用 (如 10ms 定时器中断)
 * @param  无。
 * @retval 无。
 * @note   根据上次计数值计算速度
 *         WARNING：采样周期内编码器增量不能超过 int16_t 表示范围，否则无法正确处理回绕。
 */
void Encoder_UpdateSpeed(void);

/**
 * @brief  清除指定电机编码器计数
 * @param  motor_id：电机编号 0~3。
 * @retval 无。
 * @note   会同步清零软件缓存和对应 TIMx->CNT 寄存器。
 *         WARNING：建议在电机停止时调用，转动中清零可能造成下一次速度估算突变。
 */
void Encoder_ClearCount(uint8_t motor_id);

/**
 * @brief  清除所有电机编码器计数
 * @param  无。
 * @retval 无。
 * @note   依次清除 M1~M4，常用于初始化或重新设定里程零点。
 */
void Encoder_ClearAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
