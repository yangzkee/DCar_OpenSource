/**
 ******************************************************************************
 * @file    imu.c
 * @brief   ICM20602 六轴 IMU 的 SPI 通信、初始化和姿态解算实现。
 *
 * 本文件通过 SPI2 访问 ICM20602 寄存器，读取加速度计、陀螺仪和温度原始
 * 数据，并使用 Mahony 互补滤波估算 roll/pitch，yaw 由 Z 轴角速度积分得到。
 *
 * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
 * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
 * 外设映射 : SPI2 主机模式，CS_ICM=PA8 手动片选
 * SPI 配置  : CPOL=1、CPHA=2EDGE、Prescaler=2，推测用于 ICM20602 SPI Mode 3
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
/* 头文件说明：
 * imu.h   : 定义 ICM20602_Data_TypeDef、ICM20602_Attitude_TypeDef、初始化状态枚举和 API。
 * stdio.h : 标准输入输出头文件；当前文件未直接使用，保留用于调试 printf。
 */
#include "imu.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* main.h : 提供 CS_ICM_Pin/CS_ICM_GPIO_Port 等 GPIO 宏定义。
 * spi.h  : 提供 hspi2 句柄和 SPI2 初始化声明。
 */
#include "main.h"
#include "spi.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* ICM20602 设备识别寄存器。
 * WHO_AM_I 寄存器地址 0x75，正常读值为 0x12。
 * 用于上电通信检查，若读值不匹配通常表示 SPI 模式、片选、供电或器件型号异常。
 */
#define ICM20602_WHO_AM_I_REG 0x75
#define ICM20602_WHO_AM_I_VAL 0x12 /* ICM20602 的默认 ID */

/* Private macro -------------------------------------------------------------*/
/* ICM20602 片选控制宏。
 * CS 低电平有效：读写寄存器前拉低，传输结束后拉高。
 * WARNING：SPI 多从机共总线时，必须确保其它从机 CS 保持高电平，避免总线冲突。
 */
#define ICM_CS_LOW()                                                           \
  HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin, GPIO_PIN_RESET)
#define ICM_CS_HIGH()                                                          \
  HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin, GPIO_PIN_SET)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint8_t SPI2_ReadWriteByte(uint8_t byte);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  向 ICM20602 寄存器写入一个字节
 * @param  reg：寄存器地址，低 7 位有效，最高位由函数清零表示写操作。
 * @param  data：要写入的 8 位数据。
 * @retval 无。
 * @note   ICM20602 SPI 访问约定：首字节 bit7=0 表示写，bit[6:0] 为寄存器地址。
 *         函数内执行 CS 拉低 -> 发送地址 -> 发送数据 -> CS 拉高。
 */
void ICM20602_WriteReg(uint8_t reg, uint8_t data) {
  ICM_CS_LOW();
  SPI2_ReadWriteByte(reg & 0x7F); /* 最高位置0表示写入 */
  SPI2_ReadWriteByte(data);
  ICM_CS_HIGH();
}

/**
 * @brief  向 ICM20602 寄存器读取一个字节
 * @param  reg：寄存器地址，低 7 位有效，最高位由函数置 1 表示读操作。
 * @retval uint8_t：读取到的寄存器值。
 * @note   读操作第二个字节发送 dummy byte 0xFF，同时从 MISO 获取返回数据。
 */
uint8_t ICM20602_ReadReg(uint8_t reg) {
  uint8_t res;
  ICM_CS_LOW();
  SPI2_ReadWriteByte(reg | 0x80); /* 最高位置1表示读取 */
  res = SPI2_ReadWriteByte(0xFF); /* 发送 dummy byte 获取数据 */
  ICM_CS_HIGH();
  return res;
}

/**
 * @brief  SPI2 读写一个字节 (基础阻塞方式)
 * @param  byte：要从 MOSI 发送的字节。
 * @retval uint8_t：同一 SPI 时钟下从 MISO 接收的字节。
 * @note   HAL_SPI_TransmitReceive() 超时时间为 1 ms。该函数为阻塞调用，
 *         不建议在高优先级中断中大量连续使用。
 */
static uint8_t SPI2_ReadWriteByte(uint8_t byte) {
  uint8_t res;
  HAL_SPI_TransmitReceive(&hspi2, &byte, &res, 1, 1);
  return res;
}

/**
 * @brief  检查 ICM20602 是否正常连接
 * @param  无。
 * @retval 1：WHO_AM_I 匹配，通信成功。
 * @retval 0：WHO_AM_I 不匹配，通信失败。
 * @note   该检查不代表量程配置已完成，只验证 SPI 读寄存器链路和芯片 ID。
 */
uint8_t ICM20602_Check(void) {
  uint8_t id = ICM20602_ReadReg(ICM20602_WHO_AM_I_REG);
  if (id == ICM20602_WHO_AM_I_VAL) {
    return 1;
  }
  return 0;
}

/* 非阻塞初始化状态机内部状态。
 * current_state : 当前 IMU 初始化阶段，见 IMU_InitState_t。
 * state_tick    : 当前阶段进入时的 HAL_GetTick() 时间戳，单位 ms。
 */
static IMU_InitState_t current_state = IMU_STATE_RESET;
static uint32_t state_tick = 0;

/**
 * @brief  ICM20602 非阻塞式初始化状态机。
 * @param  无。
 * @retval IMU_InitState_t：当前初始化状态。
 * @note   状态流：
 *         RESET -> WAIT_RESET(100 ms) -> WAKEUP -> WAIT_WAKEUP(50 ms)
 *         -> CONFIG -> READY；检测失败进入 ERROR，500 ms 后重试。
 *         寄存器说明：
 *         - 0x6B PWR_MGMT_1：0x80 复位，0x01 唤醒并选择 PLL/自动时钟源；
 *         - 0x1B GYRO_CONFIG：0x18 = bit[4:3]=11，陀螺仪量程 ±2000 dps；
 *         - 0x1C ACCEL_CONFIG：0x18 = bit[4:3]=11，加速度量程 ±16 g；
 *         - 0x1D ACCEL_CONFIG2：0x06，配置加速度低通滤波。
 *         WARNING：初始化状态机会在每次调用时推进，调用频率过低会延长启动时间。
 */
IMU_InitState_t ICM20602_Init_NonBlocking(void) {
  uint32_t now = HAL_GetTick();

  switch (current_state) {
  case IMU_STATE_RESET:
    ICM20602_WriteReg(0x6B, 0x80); // 发送复位
    state_tick = now;
    current_state = IMU_STATE_WAIT_RESET;
    break;

  case IMU_STATE_WAIT_RESET:
    if (now - state_tick >= 100) { // 等待复位完成
      current_state = IMU_STATE_WAKEUP;
    }
    break;

  case IMU_STATE_WAKEUP:
    ICM20602_WriteReg(0x6B, 0x01); // 唤醒，选择自动时钟源
    state_tick = now;
    current_state = IMU_STATE_WAIT_WAKEUP;
    break;

  case IMU_STATE_WAIT_WAKEUP:
    if (now - state_tick >= 50) { // 等待唤醒稳定
      if (ICM20602_Check()) {
        current_state = IMU_STATE_CONFIG;
      } else {
        current_state = IMU_STATE_ERROR;
      }
    }
    break;

  case IMU_STATE_CONFIG:
    // 基础量程配置
    ICM20602_WriteReg(0x1B, 0x18); // 陀螺仪：+-2000dps
    ICM20602_WriteReg(0x1C, 0x18); // 加速度计：+-16g
    ICM20602_WriteReg(0x1D, 0x06); // 加速度计低通滤波
    current_state = IMU_STATE_READY;
    break;

  case IMU_STATE_READY:
    /* 正常运行状态，保持 */
    break;

  case IMU_STATE_ERROR:
    /* 错误状态：等待 500ms 后自动重试复位 */
    if (now - state_tick >= 500) {
      current_state = IMU_STATE_RESET;
      state_tick = now;
    }
    break;
  }
  return current_state;
}

/**
 * @brief  初始化 ICM20602 (传统阻塞方式，保留备用)
 * @param  无。
 * @retval 1：初始化成功。
 * @retval 0：WHO_AM_I 检查失败。
 * @note   使用 HAL_Delay() 等待复位和唤醒，适合上电一次性初始化；
 *         当前主循环使用非阻塞状态机，避免长时间阻塞控制任务。
 */
uint8_t ICM20602_Init(void) {
  // 1. 复位设备
  ICM20602_WriteReg(0x6B, 0x80);
  HAL_Delay(100);

  // 2. 解除睡眠，选择自动选择时钟源
  ICM20602_WriteReg(0x6B, 0x01);
  HAL_Delay(50);

  // 3. 检查 ID
  if (!ICM20602_Check()) {
    return 0;
  }

  // 4. 配置陀螺仪/加速度计量程 (基础设置)
  ICM20602_WriteReg(0x1B, 0x18); // 陀螺仪: +-2000dps
  ICM20602_WriteReg(0x1C, 0x18); // 加速度计: +-16g
  ICM20602_WriteReg(0x1D, 0x06); // 加速度计低通滤波

  return 1;
}

#include <math.h>

/* Mahony 互补滤波参数。
 * Kp    : 比例增益，用于快速根据重力误差修正陀螺仪漂移。
 * Ki    : 积分增益，用于累积补偿长期零漂。
 * halfT : 采样周期的一半，当前假设 IMU 更新频率 1 kHz，T=0.001 s。
 */
#define Kp 2.0f       /* 比例增益控制加速度计补偿陀螺仪漂移的速度 */
#define Ki 0.005f     /* 积分增益控制漂移补偿的精确度 */
#define halfT 0.0005f /* 采样周期的一半 (假设主循环中 IMU 更新频率为 1kHz) */

/* Mahony 四元数和积分误差项。
 * q0~q3        : 姿态四元数，初始为单位四元数，表示无旋转。
 * integral_e*  : 加速度重力方向误差的积分项，用于补偿陀螺漂移。
 * 使用限制：这些变量为全局可见，若多个文件同时访问需要避免并发写入。
 */
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integral_ex = 0.0f, integral_ey = 0.0f, integral_ez = 0.0f;

/**
 * @brief  Mahony 互补滤波及四元数更新
 * @param  raw：ICM20602 原始数据指针，包含加速度、陀螺仪和温度原始 ADC 值。
 * @param  angle：输出姿态角指针，roll/pitch/yaw 单位为 degree。
 * @retval 无。
 * @note   陀螺仪换算：
 *         ±2000 dps 量程下 1 LSB ≈ 2000/32768 = 0.061035 dps；
 *         转为 rad/s 约 0.061035*pi/180 = 0.0010653。
 *         yaw 当前使用 Z 轴角速度积分，并设置 ±3 dps 死区抑制零漂。
 *         WARNING：Mahony 算法无法仅靠加速度修正 yaw，长期 yaw 会受陀螺零漂影响。
 */
void ICM20602_UpdateAttitude(ICM20602_Data_TypeDef *raw,
                             ICM20602_Attitude_TypeDef *angle) {
  float ax = raw->acc_x;
  float ay = raw->acc_y;
  float az = raw->acc_z;
  float gx = raw->gyro_x * 0.0010653f; /* 转化为弧度/秒 */
  float gy = raw->gyro_y * 0.0010653f;
  float gz = raw->gyro_z * 0.0010653f;

  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  /* 1. 加速度归一化 (重力方向) */
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm > 0) {
    ax /= norm;
    ay /= norm;
    az /= norm;

    /* 2. 提取当前四元数得到的重力分量 (在地理坐标系下的投影) */
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 3. 叉乘得到误差 (测量重力 vs 预测重力) */
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    /* 4. 误差积分 */
    integral_ex += ex * Ki;
    integral_ey += ey * Ki;
    integral_ez += ez * Ki;

    /* 5. 使用 Kp 和 Ki 修正角速度 */
    gx = gx + Kp * ex + integral_ex;
    gy = gy + Kp * ey + integral_ey;
    gz = gz + Kp * ez + integral_ez;
  }

  /* 6. 四元数微分方程更新 */
  q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
  q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT;
  q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT;
  q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT;

  /* 7. 四元数归一化 */
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;

  /* 8. 转换为欧拉角 (角度制) */
  angle->roll =
      atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) *
      57.29578f;
  angle->pitch = -asin(2 * (q1 * q3 - q0 * q2)) * 57.29578f;

  /* 9. Yaw 轴特殊处理: 条件积分 (Deadzone) */
  /* 说明: 这种写法通常用于滤除零飘，认为当转速大于 3 或小于 -3 时才开始积分更新
   * Yaw */
  float gz_deg = raw->gyro_z * 0.061035f; // 转化到 度/秒 (2000/32768)
  if (gz_deg > 3.0f || gz_deg < -3.0f) {
    // 0.001f 为时钟周期 (1ms)，这里假设本函数被 1kHz 周期调用
    angle->yaw -= (gz_deg * 0.001f);
  }
}

/**
 * @brief  读取 IMU 原始数据 (加速度、温度、陀螺仪)
 * @param  data：输出数据结构体指针，保存 7 个 int16_t 原始值。
 * @retval 无。
 * @note   从寄存器 0x3B 开始 Burst Read 14 字节：
 *         ACCEL_X/Y/Z 高低字节、TEMP 高低字节、GYRO_X/Y/Z 高低字节。
 *         ICM20602 数据为大端格式，即高字节在前。
 */
void ICM20602_ReadData(ICM20602_Data_TypeDef *data) {
  uint8_t buf[14];

  ICM_CS_LOW();
  SPI2_ReadWriteByte(0x3B | 0x80); // 从加速度 X 轴高位地址开始读

  // 连续读取 14 个字节 (Burst Read)
  for (int i = 0; i < 14; i++) {
    buf[i] = SPI2_ReadWriteByte(0xFF);
  }
  ICM_CS_HIGH();

  // 数据拼凑 (高位在前)
  data->acc_x = (int16_t)((buf[0] << 8) | buf[1]);
  data->acc_y = (int16_t)((buf[2] << 8) | buf[3]);
  data->acc_z = (int16_t)((buf[4] << 8) | buf[5]);
  data->temp = (int16_t)((buf[6] << 8) | buf[7]);
  data->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
  data->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
  data->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
}
