/**
 ******************************************************************************
 * @file    motor_control.c
 * @brief   四电机 PID 速度闭环控制实现。
 *
 * 本文件把上层给出的四轮目标速度转换为 TIM8 PWM 占空比和 GPIOD 方向
 * 控制信号。速度反馈来自 encoder.c，PID 输出范围被限制在 TIM8 自动重装
 * 周期内，支持正反转和低速死区空载。
 *
 * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
 * 外设映射 : TIM8_CH1~CH4 输出四路 PWM；GPIOD PD0~PD7 控制 H 桥方向
 * 调用周期 : MotorControl_Update() 由 TIM6 定时链路每 10 ms 调用一次
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 ******************************************************************************
 */

/* 头文件说明：
 * motor_control.h : 本模块 API、MOTOR_PWM_PERIOD、默认 PID 参数声明。
 * encoder.h       : 提供编码器初始化、速度更新和线速度读取接口。
 * gpio.h          : 提供 GPIO 初始化声明；方向引脚宏实际定义于 main.h。
 * pid.h           : 提供 PID_TypeDef、PID_Init()、PID_Calc()、PID_Reset()。
 * tim.h           : 提供 htim8 PWM 句柄和 htim6 基本定时器句柄。
 * math.h          : 提供 fabsf()，用于停止死区判断。
 */
#include "motor_control.h"
#include "encoder.h"
#include "gpio.h"
#include "pid.h"
#include "tim.h"
#include <math.h>

/* 电机方向引脚表。
 * motor_in1_pin/motor_in2_pin：每个电机 H 桥的两路方向输入，均位于 GPIOD。
 * 逻辑约定：
 * - IN1=1、IN2=0：正转；
 * - IN1=0、IN2=1：反转；
 * - IN1=0、IN2=0：空载滑行。
 * WARNING：若实际驱动板为刹车逻辑或高有效/低有效不同，需要同步调整 Motor_SetPWM()。
 */
static const uint16_t motor_in1_pin[] = {AIN1_Pin, BIN1_Pin, CIN1_Pin,
                                         DIN1_Pin};
static const uint16_t motor_in2_pin[] = {AIN2_Pin, BIN2_Pin, CIN2_Pin,
                                         DIN2_Pin};
static GPIO_TypeDef *motor_gpio_port = GPIOD;

/* 四个电机目标速度，单位 count/s。
 * 初始值为 0，表示上电后电机保持停止。
 * 上层若以 mm/s 设置目标，会在 MotorControl_SetAllTargetSpeedMMps()
 * 中乘以 COUNTS_PER_MM 转换为 count/s。
 */
static float target_speed[4] = {0};

/* 四个电机独立 PID 控制器。
 * 每个控制器的输入量纲为“每 10 ms 的编码器增量”，输出量纲为 PWM 比较值。
 */
static PID_TypeDef motor_pid[4];

/* M1、M4 方向取反（电机安装方向与 M2、M3 相反）。
 * 返回值只可能为 +1 或 -1，用于统一上层运动学正方向。
 */
#define MOTOR_DIR_INVERT(m) (((m) == 0 || (m) == 3) ? -1 : 1)

/* 速度(count/s) -> 每周期计数(count/10ms)。
 * 由于 ENCODER_SAMPLE_MS=10 ms，1 秒包含 100 个采样周期，因此除以 100。
 */
#define SPEED_TO_DELTA(s) ((s) / 100.0f)

/* PWM 和停止相关阈值。
 * MOTOR_PWM_DEADZONE          : PWM 绝对值小于 60 时直接空载，避免低占空比无法克服静摩擦导致啸叫。
 * MOTOR_TARGET_STOP_DEADBAND  : 目标速度小于 3 mm/s 对应的 count/s 时认为目标为 0。
 * MOTOR_SPEED_STOP_DEADBAND   : 当前速度小于 300 count/s 时配合目标死区触发停止。
 */
#define MOTOR_PWM_DEADZONE 60
#define MOTOR_TARGET_STOP_DEADBAND (COUNTS_PER_MM * 3.0f)
#define MOTOR_SPEED_STOP_DEADBAND 300.0f

/**
 * @brief  设置指定电机 PWM 通道的比较值。
 * @param  motor_id：电机编号，0~3 对应 TIM8_CH1~TIM8_CH4。
 * @param  pwm_abs：PWM 比较值，范围建议 0~MOTOR_PWM_PERIOD。
 * @retval 无。
 * @note   仅写 TIM8 CCRx，不改变方向引脚。TIM8 周期为 999，
 *         占空比约为 pwm_abs/(999+1)。
 */
static void Motor_SetCompare(uint8_t motor_id, uint16_t pwm_abs) {
  switch (motor_id) {
  case 0:
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm_abs);
    break;
  case 1:
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm_abs);
    break;
  case 2:
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm_abs);
    break;
  case 3:
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm_abs);
    break;
  default:
    break;
  }
}

/**
 * @brief  让指定电机空载滑行。
 * @param  motor_id：电机编号，0~3。
 * @retval 无。
 * @note   PWM 比较值清零，同时 IN1/IN2 均置低。
 *         WARNING：空载滑行不等同于电气刹车，车辆可能因惯性继续移动。
 */
static void Motor_Coast(uint8_t motor_id) {
  if (motor_id >= 4)
    return;

  Motor_SetCompare(motor_id, 0);
  HAL_GPIO_WritePin(motor_gpio_port, motor_in1_pin[motor_id], GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor_gpio_port, motor_in2_pin[motor_id], GPIO_PIN_RESET);
}

/**
 * @brief  设置电机 PWM 占空比并同步控制转向。
 * @param  motor_id：电机编号，0~3。
 * @param  pwm_val：带符号 PWM 输出，范围建议 -999~+999；正负表示方向。
 * @retval 无。
 * @note   内部先按 MOTOR_DIR_INVERT() 修正安装方向，再根据符号设置 H 桥方向。
 *         位/极性说明：
 *         - pwm_val >= 0：IN1=1、IN2=0；
 *         - pwm_val <  0：IN1=0、IN2=1；
 *         - |pwm_val| < MOTOR_PWM_DEADZONE：IN1=0、IN2=0 且 PWM=0。
 */
static void Motor_SetPWM(uint8_t motor_id, int16_t pwm_val) {
  if (motor_id >= 4)
    return;

  pwm_val *= MOTOR_DIR_INVERT(motor_id);

  uint16_t pwm_abs;
  /* PWM 输出死区：小占空比直接空载，防止低压啸叫 */
  if (pwm_val > -MOTOR_PWM_DEADZONE && pwm_val < MOTOR_PWM_DEADZONE) {
    Motor_Coast(motor_id);
    return;
  }

  if (pwm_val >= 0) {
    pwm_abs =
        (pwm_val > MOTOR_PWM_PERIOD) ? MOTOR_PWM_PERIOD : (uint16_t)pwm_val;
    HAL_GPIO_WritePin(motor_gpio_port, motor_in1_pin[motor_id], GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_gpio_port, motor_in2_pin[motor_id], GPIO_PIN_RESET);
  } else {
    pwm_abs =
        (-pwm_val > MOTOR_PWM_PERIOD) ? MOTOR_PWM_PERIOD : (uint16_t)(-pwm_val);
    HAL_GPIO_WritePin(motor_gpio_port, motor_in1_pin[motor_id], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_gpio_port, motor_in2_pin[motor_id], GPIO_PIN_SET);
  }

  Motor_SetCompare(motor_id, pwm_abs);
}

/**
 * @brief  初始化电机速度闭环模块。
 * @param  无。
 * @retval 无。
 * @note   初始化顺序：
 *         1. 启动四路编码器接口；
 *         2. 启动 TIM6 基本定时器中断；
 *         3. 启动 TIM8 四路 PWM；
 *         4. 初始化四个 PID 并让电机空载。
 *         WARNING：调用前必须完成 MX_TIM2/3/4/5/6/8_Init() 和 MX_GPIO_Init()。
 */
void MotorControl_Init(void) {
  Encoder_Init();

  /* 启动 TIM6 定时中断 (1ms)，用于 10ms 周期调用 MotorControl_Update */
  HAL_TIM_Base_Start_IT(&htim6);

  /* 启动 PWM */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  /* 初始化 PID，输出范围 -999 ~ 999 (支持正反转) */
  for (uint8_t i = 0; i < 4; i++) {
    PID_Init(&motor_pid[i], MOTOR_PID_KP_DEFAULT, MOTOR_PID_KI_DEFAULT,
             MOTOR_PID_KD_DEFAULT, (float)MOTOR_PWM_PERIOD,
             -(float)MOTOR_PWM_PERIOD);
    target_speed[i] = 0.0f;
    Motor_Coast(i);
  }
}

/**
 * @brief  设置单个电机目标速度。
 * @param  motor_id：电机编号，0~3。
 * @param  target_speed_val：目标速度，单位 count/s，正负表示统一后的运动学方向。
 * @retval 无。
 * @note   非法 motor_id 会被忽略。
 */
void MotorControl_SetTargetSpeed(uint8_t motor_id, float target_speed_val) {
  if (motor_id < 4) {
    target_speed[motor_id] = target_speed_val;
  }
}

/**
 * @brief  直接设置四个电机的目标编码器速度。
 * @param  s1：M1 目标速度，单位 count/s。
 * @param  s2：M2 目标速度，单位 count/s。
 * @param  s3：M3 目标速度，单位 count/s。
 * @param  s4：M4 目标速度，单位 count/s。
 * @retval 无。
 * @note   本接口面向底层调试；正常运动控制优先使用 mm/s 接口。
 */
void MotorControl_SetAllTargetSpeed(float s1, float s2, float s3, float s4) {
  target_speed[0] = s1;
  target_speed[1] = s2;
  target_speed[2] = s3;
  target_speed[3] = s4;
}

/**
 * @brief  设置四个电机的目标轮端线速度。
 * @param  mm_s1：M1 目标线速度，单位 mm/s。
 * @param  mm_s2：M2 目标线速度，单位 mm/s。
 * @param  mm_s3：M3 目标线速度，单位 mm/s。
 * @param  mm_s4：M4 目标线速度，单位 mm/s。
 * @retval 无。
 * @note   换算公式：target_count_per_sec = speed_mmps * COUNTS_PER_MM。
 */
void MotorControl_SetAllTargetSpeedMMps(float mm_s1, float mm_s2, float mm_s3,
                                        float mm_s4) {
  target_speed[0] = mm_s1 * COUNTS_PER_MM;
  target_speed[1] = mm_s2 * COUNTS_PER_MM;
  target_speed[2] = mm_s3 * COUNTS_PER_MM;
  target_speed[3] = mm_s4 * COUNTS_PER_MM;
}

/**
 * @brief  执行一次四电机速度 PID 闭环更新。
 * @param  无。
 * @retval 无。
 * @note   流程：
 *         1. 调用 Encoder_UpdateSpeed() 刷新 count/s 速度。
 *         2. 将目标速度和当前速度转换为每 10 ms 计数增量。
 *         3. PID 输出带符号 PWM，传给 Motor_SetPWM()。
 *         4. 当目标和当前速度都进入停止死区时，复位 PID 并空载。
 *         WARNING：该函数在定时中断链路中执行时，不应加入阻塞式串口打印。
 */
void MotorControl_Update(void) {
  Encoder_UpdateSpeed();

  for (uint8_t i = 0; i < 4; i++) {
    float current_speed = Encoder_GetSpeed(i) * MOTOR_DIR_INVERT(i);
    if (fabsf(target_speed[i]) < MOTOR_TARGET_STOP_DEADBAND &&
        fabsf(current_speed) < MOTOR_SPEED_STOP_DEADBAND) {
      target_speed[i] = 0.0f;
      PID_Reset(&motor_pid[i]);
      Motor_Coast(i);
      continue;
    }
    /* 转为每周期脉冲 (脉冲/10ms)，与参考工程量纲一致 */
    float target_delta = SPEED_TO_DELTA(target_speed[i]);
    float current_delta = SPEED_TO_DELTA(current_speed);
    float out = PID_Calc(&motor_pid[i], target_delta, current_delta);
    Motor_SetPWM(i, (int16_t)out);
  }
}

/**
 * @brief  重新设置指定电机 PID 参数。
 * @param  motor_id：电机编号，0~3。
 * @param  kp：比例增益。
 * @param  ki：积分增益。
 * @param  kd：微分增益。
 * @retval 无。
 * @note   会重新初始化该电机 PID，历史积分和误差被清零。
 */
void MotorControl_SetPID(uint8_t motor_id, float kp, float ki, float kd) {
  if (motor_id < 4) {
    PID_Init(&motor_pid[motor_id], kp, ki, kd, (float)MOTOR_PWM_PERIOD,
             -(float)MOTOR_PWM_PERIOD);
  }
}

/**
 * @brief  停止所有电机并复位速度 PID。
 * @param  无。
 * @retval 无。
 * @note   清零目标速度、复位 PID、四路电机空载。用于 IMU 未就绪、错误保护或急停。
 */
void MotorControl_StopAll(void) {
  for (uint8_t i = 0; i < 4; i++) {
    target_speed[i] = 0.0f;
    PID_Reset(&motor_pid[i]);
    Motor_Coast(i);
  }
}

/**
 * @brief  获取指定电机当前编码器速度。
 * @param  motor_id：电机编号，0~3。
 * @retval float：速度，单位 count/s；非法编号返回 0.0f。
 * @note   返回值已经乘以 MOTOR_DIR_INVERT()，方向与上层运动学一致。
 */
float MotorControl_GetSpeed(uint8_t motor_id) {
  if (motor_id >= 4)
    return 0.0f;
  return Encoder_GetSpeed(motor_id) * MOTOR_DIR_INVERT(motor_id);
}

/**
 * @brief  获取指定电机当前轮端线速度。
 * @param  motor_id：电机编号，0~3。
 * @retval float：线速度，单位 mm/s；非法编号返回 0.0f。
 * @note   返回值已经统一电机安装方向，可直接用于里程计反解。
 */
float MotorControl_GetSpeedMMps(uint8_t motor_id) {
  if (motor_id >= 4)
    return 0.0f;
  return Encoder_GetSpeedMMps(motor_id) * MOTOR_DIR_INVERT(motor_id);
}
