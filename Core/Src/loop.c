/**
 ******************************************************************************
 * @file    loop.c
 * @brief   主循环任务分频调度实现
 *
 * 本文件基于 HAL_GetTick() 的 1 ms 系统节拍，在 while(1) 中以软件分频
 * 的方式调度 IMU 更新、运动控制、遥控/串口命令处理和调试输出。
 *
 * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
 * 时间基准 : SysTick 1 ms，由 HAL_Init() 配置
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 *
 *  分频结构 (基于 HAL_GetTick 软件计时):
 *  ┌─────────────┬──────────┬──────────────────────────────────────────┐
 *  │  函数        │  周期    │  任务                                    │
 *  ├─────────────┼──────────┼──────────────────────────────────────────┤
 *  │ LOOP_1000HZ │  1  ms   │ IMU 读取 + Mahony 姿态解算               │
 *  │ LOOP_100HZ  │  10 ms   │ 遥控模式判断 + Yaw PID + 运动解耦        │
 *  │ LOOP_50HZ   │  20 ms   │ UART4 调试输出 (VOFA+)                   │
 *  └─────────────┴──────────┴──────────────────────────────────────────┘
 *
 *  注: 电机速度 PID (100Hz) 由 TIM6 硬件中断独立驱动，不在此调度，
 *      见 stm32f4xx_it.c → HAL_TIM_PeriodElapsedCallback。
 ******************************************************************************
 */
/* 头文件说明：
 * loop.h             : 本模块对外入口 Loop_Run()。
 * gpio.h/main.h      : GPIO 宏与 HAL GPIO 控制接口，包含 LED0、PC13 调试引脚等。
 * imu.h              : ICM20602 初始化、数据读取与姿态解算接口。
 * motor_control.h    : 电机急停接口 MotorControl_StopAll()。
 * move_control.h     : 相对位移任务状态和更新接口。
 * odometry.h         : 里程计更新接口。
 * ps2_receiver.h     : PS2/SBUS 遥控数据读取接口。
 * remote_to_motion.h : 手动速度和锁头控制接口 Motion_HandleManual()。
 * uart4_debug.h      : UART4 调试输出接口，当前调试输出大部分被 #if 0 关闭。
 * usart1_control.h   : USART1 上位机速度/位移命令接口。
 * math.h             : fabsf()，用于摇杆死区判断。
 * stdint.h           : 固定宽度整型。
 * stdio.h/string.h   : snprintf() 等调试输出相关函数。
 * usart.h            : huart4 句柄，供调试串口发送使用。
 */
#include "loop.h"

#include "gpio.h"
#include "imu.h"
#include "main.h"
#include "motor_control.h"
#include "move_control.h"
#include "odometry.h"
#include "ps2_receiver.h"
#include "remote_to_motion.h"
#include "uart4_debug.h"
#include "usart1_control.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <usart.h>
#include <string.h>
/* -------------------------------------------------------------------------- */
/* 模块内共享状态 */
/* -------------------------------------------------------------------------- */
/* imu_raw             : 最近一次读取到的 ICM20602 原始加速度/陀螺仪数据。
 * imu_angles          : Mahony/积分计算得到的姿态角，单位 degree。
 * imu_status          : IMU 非阻塞初始化状态，只有 READY 后才允许运动控制。
 * remote_mode_enabled : 遥控使能标志，CH6 处于约 600 附近时置 1。
 */
static ICM20602_Data_TypeDef imu_raw;
static ICM20602_Attitude_TypeDef imu_angles = {0};
static IMU_InitState_t imu_status = IMU_STATE_RESET;
static uint8_t remote_mode_enabled = 0;
//static uint32_t total_ticks = 0;

/* -------------------------------------------------------------------------- */
/* 1000 Hz — IMU 读取 + Mahony 姿态解算                                       */
/* -------------------------------------------------------------------------- */
/**
 * @brief  1 kHz IMU 任务。
 * @param  无。
 * @retval 无。
 * @note   每 1 ms 调用一次非阻塞初始化状态机。IMU_READY 后读取原始数据并更新姿态角。
 *         WARNING：ICM20602_UpdateAttitude() 内部假设采样周期为 1 ms，
 *         若本任务调度频率改变，需要同步调整 halfT/yaw 积分周期。
 */
static void LOOP_1000HZ(void) {
  imu_status = ICM20602_Init_NonBlocking();

  if (imu_status == IMU_STATE_READY) {
    ICM20602_ReadData(&imu_raw);
    ICM20602_UpdateAttitude(&imu_raw, &imu_angles);
  }
}

/* -------------------------------------------------------------------------- */
/* 100 Hz — 遥控/串口模式判断 + Yaw PID + 运动解耦                            */
/* -------------------------------------------------------------------------- */
/**
 * @brief  100 Hz 运动控制任务。
 * @param  无。
 * @retval 无。
 * @note   控制优先级：
 *         1. USART1 速度帧直接下发 vx/vy/w。
 *         2. USART1 位移帧设置 MoveControl 相对目标，并清除命令有效位。
 *         3. 位移任务执行中由 MoveControl_Update() 接管四轮速度。
 *         4. 非位移任务且无串口速度时读取 PS2/SBUS 遥控。
 *         5. IMU 未 READY 时立即 MotorControl_StopAll()。
 */
static void LOOP_100HZ(void) {
  /* ===== 调试：入口翻转 PC13 ===== */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  
  Uart1_ControlCmd_t uart_cmd;
  float vx = 0, vy = 0, w = 0;
  uint8_t manual_active = 0;

  if (imu_status == IMU_STATE_READY) {
    /* ===== 调试：步骤1 ===== */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    
    Uart1_Control_GetLatestCmd(&uart_cmd);

    /* 1. 串口处理：串口命令优先级高于遥控输入。 */
    if (uart_cmd.valid) {
      if (uart_cmd.type == UART1_CMD_VELOCITY) {
        vx = uart_cmd.vx_mmps;
        vy = uart_cmd.vy_mmps;
        w = uart_cmd.w_mmps;
        manual_active = 1;
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
      } else if (uart_cmd.type == UART1_CMD_DISPLACEMENT) {
        MoveControl_SetRelativeTarget(uart_cmd.target_x_mm,
                                      uart_cmd.target_y_mm, imu_angles.yaw,
                                      uart_cmd.target_speed_mmps);
        Uart1_Control_ClearCmd();
      }
    }

    /* ===== 调试：步骤2 ===== */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    /* 2. 遥控处理：仅在没有串口速度指令且未执行位移任务时生效。 */
    if (!manual_active && MoveControl_GetState() != MOVE_EXECUTING) {
      PS2_Data_TypeDef data;
      PS2_Receiver_GetData(&data);

      /* 更新遥控开关指示灯。
       * 当前判断 CH6 在 [550,650] 区间时启用遥控模式，推测为接收机某个按键/拨杆中位。
       */
      if (data.ch6 >= 550 && data.ch6 <= 650) {
        remote_mode_enabled = 1;
      }
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,
                        remote_mode_enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);

      if (remote_mode_enabled && data.connected) {
        /* 使用线性映射将 SBUS 原始摇杆值转换为 mm/s。 */
        float ly_speed = ((float)((int32_t)data.ly - REMOTE_CENTER)) *
                         REMOTE_MAX_SPEED / REMOTE_RANGE;
        float lx_speed = ((float)((int32_t)data.lx - REMOTE_CENTER)) *
                         REMOTE_MAX_SPEED / REMOTE_RANGE;
        float rx_speed = ((float)((int32_t)data.rx - REMOTE_CENTER)) *
                         REMOTE_MAX_SPEED / REMOTE_RANGE;

        /* 应用 20 mm/s 死区，滤除摇杆回中抖动。 */
        if (fabsf(ly_speed) > 20.0f)
          vy = ly_speed;
        if (fabsf(lx_speed) > 20.0f)
          vx = lx_speed;
        if (fabsf(rx_speed) > 20.0f)
          w = rx_speed;
      }
    }

    /* ===== 调试：步骤3 ===== */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    /* 3. 任务执行与锁头下发：位移任务执行中优先，否则按手动速度控制。 */
    if (MoveControl_GetState() == MOVE_EXECUTING) {
      MoveControl_Update();
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,
                        GPIO_PIN_RESET); // 移动中 LED 常亮
    } else {
      /* Motion_HandleManual 内部包含锁头逻辑 */
      Motion_HandleManual(vx, vy, w, imu_angles.yaw);
    }

    /* ===== 调试：步骤4 ===== */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    /* 发送串口应答：实际发送在主循环中完成，避免在 UART 回调中阻塞。 */
    Uart1_Control_SendAck();

    /* ===== 调试：步骤5 ===== */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    /* 更新里程计：100 Hz 调用，因此 dt=0.01 s。 */
    Odometry_Update(imu_angles.yaw, 0.01f);
  } else {
    MotorControl_StopAll();
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  }
  
  /* ===== 调试：出口翻转 PC13 ===== */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/* -------------------------------------------------------------------------- */
/* 50 Hz — UART4 调试输出 (VOFA+)                                             */
/* -------------------------------------------------------------------------- */
/**
 * @brief  50 Hz 调试输出任务。
 * @param  无。
 * @retval 无。
 * @note   当前函数在 IMU_READY 时立即 return，后续 #if 0 代码保留为调试模板。
 *         若启用 UART 打印，应确认 50 Hz 输出长度不会占用过多主循环时间。
 */
static void LOOP_50HZ(void) {
  if (imu_status == IMU_STATE_READY) {
    return;
#if 0

    /* 打印里程计数据，验证方向正负 */
    Odometry_TypeDef odo_data;
    Odometry_GetData(&odo_data);

    uint8_t enc1_a = Read_GPIO_Bit(GPIOA, GPIO_PIN_5);
    uint8_t enc1_b = Read_GPIO_Bit(GPIOB, GPIO_PIN_3);
    uint8_t enc2_a = Read_GPIO_Bit(GPIOA, GPIO_PIN_6);
    uint8_t enc2_b = Read_GPIO_Bit(GPIOA, GPIO_PIN_7);
    uint8_t enc3_a = Read_GPIO_Bit(GPIOB, GPIO_PIN_6);
    uint8_t enc3_b = Read_GPIO_Bit(GPIOB, GPIO_PIN_7);
    uint8_t enc4_a = Read_GPIO_Bit(GPIOA, GPIO_PIN_0);
    uint8_t enc4_b = Read_GPIO_Bit(GPIOA, GPIO_PIN_1);

    int len = 0;
    char buf[192];
    len = snprintf(buf, sizeof(buf),
                   "ODO: x:%.1f y:%.1f yaw:%.1f vx:%.1f vy:%.1f "
                   "ENC_AB: M1:%u%u M2:%u%u M3:%u%u M4:%u%u\n",
                   odo_data.x, odo_data.y, odo_data.yaw, odo_data.vx,
                   odo_data.vy, (unsigned)enc1_a, (unsigned)enc1_b,
                   (unsigned)enc2_a, (unsigned)enc2_b, (unsigned)enc3_a,
                   (unsigned)enc3_b, (unsigned)enc4_a, (unsigned)enc4_b);
    if (len > 0) {
      if (len >= (int)sizeof(buf)) {
        len = (int)sizeof(buf) - 1;
      }
      HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 10);
    }
    //printf("ODO: x:%.1f y:%.1f yaw:%.1f vx:%.1f vy:%.1f\n", odo_data.x,
           //odo_data.y, odo_data.yaw, odo_data.vx, odo_data.vy);
#endif
  }
}

/* -------------------------------------------------------------------------- */
/* 主调度入口 — 在 while(1) 中持续调用 */
/* -------------------------------------------------------------------------- */
/**
 * @brief  主循环软件分频调度入口。
 * @param  无。
 * @retval 无。
 * @note   由 main.c 的 while(1) 尽可能频繁调用。使用 HAL_GetTick() 差值判断
 *         1 ms、10 ms、20 ms 周期，天然支持 uint32_t tick 回绕。
 *         WARNING：若某个任务执行时间超过周期，会产生调度抖动；本实现不会补跑丢失周期。
 */
void Loop_Run(void) {
  static uint32_t last_1000hz = 0;
  static uint32_t last_100hz = 0;
  static uint32_t last_50hz = 0;

  uint32_t now = HAL_GetTick();

  if (now - last_1000hz >= 1) {
  last_1000hz = now; 
    LOOP_1000HZ();
  }

  if (now - last_100hz >= 10) {
    last_100hz = now;
    LOOP_100HZ();
  }

  if (now - last_50hz >= 20) {
    last_50hz = now;
    LOOP_50HZ();
  }
}
