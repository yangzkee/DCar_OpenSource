/**
 ******************************************************************************
 * @file    imu.h
 * @brief   ICM20602 六轴 IMU 驱动与姿态解算接口声明。
 *
 * 硬件平台 : STM32F407VETx，SPI2 + PA8 片选
 * 开发环境 : Keil uVision / MDK-ARM
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 ******************************************************************************
 */
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* main.h：提供 HAL 类型、GPIO 宏和 CS_ICM_Pin/CS_ICM_GPIO_Port 定义。 */
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  ICM20602 非阻塞初始化状态枚举。
 * @note   用于 ICM20602_Init_NonBlocking() 的状态机返回值。
 */
typedef enum {
  IMU_STATE_RESET = 0,   // 发送复位指令
  IMU_STATE_WAIT_RESET,  // 等待复位完成 (100ms)
  IMU_STATE_WAKEUP,      // 发送唤醒指令
  IMU_STATE_WAIT_WAKEUP, // 等待唤醒稳定 (50ms)
  IMU_STATE_CONFIG,      // 配置量程等参数
  IMU_STATE_READY,       // 已就绪
  IMU_STATE_ERROR        // 出错
} IMU_InitState_t;

/**
 * @brief  姿态角输出结构体。
 * @note   roll/pitch/yaw 单位均为 degree；yaw 当前由 Z 轴陀螺积分得到。
 */
typedef struct {
  float roll;
  float pitch;
  float yaw;
} ICM20602_Attitude_TypeDef;

/**
 * @brief  ICM20602 原始传感器数据结构体。
 * @note   各字段为传感器原始 16 位补码值，未转换为 g、dps 或摄氏度。
 */
typedef struct {
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp;
} ICM20602_Data_TypeDef;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief  阻塞式初始化 ICM20602。
 * @param  无。
 * @retval 1=成功，0=失败。
 */
uint8_t ICM20602_Init(void);

/**
 * @brief  非阻塞式初始化 ICM20602。
 * @param  无。
 * @retval IMU_InitState_t 当前状态。
 */
IMU_InitState_t ICM20602_Init_NonBlocking(void);

/**
 * @brief  读取 WHO_AM_I 并检查 ICM20602 通信是否正常。
 * @param  无。
 * @retval 1=正常，0=异常。
 */
uint8_t ICM20602_Check(void);

/**
 * @brief  读取加速度、温度和陀螺仪原始数据。
 * @param  data：输出结构体指针。
 * @retval 无。
 */
void ICM20602_ReadData(ICM20602_Data_TypeDef *data);

/**
 * @brief  根据原始 IMU 数据更新姿态角。
 * @param  raw：原始数据指针。
 * @param  angle：姿态角输出指针。
 * @retval 无。
 */
void ICM20602_UpdateAttitude(ICM20602_Data_TypeDef *raw,
                             ICM20602_Attitude_TypeDef *angle);

/**
 * @brief  读取 ICM20602 单个寄存器。
 * @param  reg：寄存器地址。
 * @retval 寄存器值。
 */
uint8_t ICM20602_ReadReg(uint8_t reg);

/**
 * @brief  写入 ICM20602 单个寄存器。
 * @param  reg：寄存器地址。
 * @param  data：写入值。
 * @retval 无。
 */
void ICM20602_WriteReg(uint8_t reg, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
