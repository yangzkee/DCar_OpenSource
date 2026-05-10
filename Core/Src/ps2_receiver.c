/**
  ******************************************************************************
  * @file    ps2_receiver.c
  * @brief   PS2 遥控器信号接收模块
  *          通过 PA3(USART2_RX) 接收 SBUS 信号，解析后直接存入变量 (0~2047)
  *
  * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
  * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
  * 晶振频率 : HSE 8 MHz，系统主频 168 MHz
  * 外设配置 : USART2，38400 baud，9-bit word，Even parity，2 stop bits
  * 协议说明 : SBUS 帧长 25 字节，帧头 0x0F，帧尾 0x00，16 路通道各 11 bit
  * 作者     : DCar_OpenSource
  * 日期     : 2026-05-10
  * 版本     : v1.0
  * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
  ******************************************************************************
  */

/* 头文件说明：
 * ps2_receiver.h : 定义 PS2_Data_TypeDef、遥控接收初始化和数据读取接口。
 * usart.h        : 提供 huart2 句柄，供 HAL_UART_Receive_IT() 启动中断接收。
 */
#include "ps2_receiver.h"
#include "usart.h"

/* SBUS 协议常量。
 * SBUS_HEADER        : 标准 SBUS 起始字节 0x0F。
 * SBUS_FOOTER        : 本工程按 0x00 作为帧尾判断。
 * SBUS_PACKET_LENGTH : SBUS 固定帧长 25 字节。
 * WARNING：部分接收机帧尾可能包含标志位或不同值，若无法解析需核对接收机协议。
 */
#define SBUS_HEADER        0x0F
#define SBUS_FOOTER        0x00
#define SBUS_PACKET_LENGTH 25

/* SBUS 字节流解析状态机。
 * WAIT_HEADER：等待帧头 0x0F。
 * RECEIVING  ：已经收到帧头，继续收满 25 字节。
 */
typedef enum {
  SBUS_STATE_WAIT_HEADER = 0,
  SBUS_STATE_RECEIVING   = 1
} SBUS_State_TypeDef;

/* 全局共享 PS2 数据。
 * 保存最近一次解析成功的 SBUS 通道值。当前由 USART2 接收完成回调写入，
 * 主循环通过 PS2_Receiver_GetData() 读取。
 */
static PS2_Data_TypeDef ps2_data;

/* SBUS 接收缓冲区和状态变量。
 * sbus_rx_buffer : 保存当前正在组装的 25 字节 SBUS 帧。
 * sbus_rx_index  : 当前写入索引，范围 0~24。
 * sbus_state     : 当前解析状态。
 */
static uint8_t  sbus_rx_buffer[SBUS_PACKET_LENGTH];
static uint8_t  sbus_rx_index;
static SBUS_State_TypeDef sbus_state;

/* 解析 SBUS 数据包并提取 16 通道 */
static uint8_t SBUS_Parse_Packet(uint8_t *packet);

/* 将 SBUS 通道存入 ps2_data (范围 0~2047 原始值) */
static void SBUS_StoreChannels(uint16_t *channels);

/**
  * @brief  解析一帧完整 SBUS 数据包。
  * @param  packet：25 字节 SBUS 帧缓冲区，packet[0] 为 0x0F，packet[24] 为帧尾。
  * @retval 1：解析成功并更新 ps2_data。
  * @retval 0：帧头/帧尾错误。
  * @note   每个通道占 11 bit，小端位拼接，解析后通道范围为 0~2047。
  *         位运算示例：CH1 = packet[1] bit[7:0] + packet[2] bit[2:0]<<8，
  *         最后统一与 0x07FF 屏蔽高位。
  */
static uint8_t SBUS_Parse_Packet(uint8_t *packet)
{
  uint16_t channels[16];

  if (packet[0] != SBUS_HEADER || packet[24] != SBUS_FOOTER)
    return 0;

  /* 解析 16 通道 (每通道 11 位) */
  channels[0]  = ((uint16_t)packet[1]) | ((uint16_t)packet[2] << 8);
  channels[0] &= 0x07FF;

  channels[1]  = ((uint16_t)packet[2] >> 3) | ((uint16_t)packet[3] << 5);
  channels[1] &= 0x07FF;

  channels[2]  = ((uint16_t)packet[3] >> 6) | ((uint16_t)packet[4] << 2) | ((uint16_t)(packet[5] & 0x01) << 10);
  channels[2] &= 0x07FF;

  channels[3]  = ((uint16_t)packet[5] >> 1) | ((uint16_t)packet[6] << 7);
  channels[3] &= 0x07FF;

  channels[4]  = ((uint16_t)packet[6] >> 4) | ((uint16_t)packet[7] << 4);
  channels[4] &= 0x07FF;

  channels[5]  = ((uint16_t)packet[7] >> 7) | ((uint16_t)packet[8] << 1) | ((uint16_t)packet[9] << 9);
  channels[5] &= 0x07FF;

  channels[6]  = ((uint16_t)packet[9] >> 2) | ((uint16_t)packet[10] << 6);
  channels[6] &= 0x07FF;

  channels[7]  = ((uint16_t)packet[10] >> 5) | ((uint16_t)packet[11] << 3);
  channels[7] &= 0x07FF;

  channels[8]  = ((uint16_t)packet[12]) | ((uint16_t)packet[13] << 8);
  channels[8] &= 0x07FF;

  channels[9]  = ((uint16_t)packet[13] >> 3) | ((uint16_t)packet[14] << 5);
  channels[9] &= 0x07FF;

  channels[10] = ((uint16_t)packet[14] >> 6) | ((uint16_t)packet[15] << 2) | ((uint16_t)packet[16] << 10);
  channels[10] &= 0x07FF;

  channels[11] = ((uint16_t)packet[16] >> 1) | ((uint16_t)packet[17] << 7);
  channels[11] &= 0x07FF;

  channels[12] = ((uint16_t)packet[17] >> 4) | ((uint16_t)packet[18] << 4);
  channels[12] &= 0x07FF;

  channels[13] = ((uint16_t)packet[18] >> 7) | ((uint16_t)packet[19] << 1) | ((uint16_t)packet[20] << 9);
  channels[13] &= 0x07FF;

  channels[14] = ((uint16_t)packet[20] >> 2) | ((uint16_t)packet[21] << 6);
  channels[14] &= 0x07FF;

  channels[15] = ((uint16_t)packet[21] >> 5) | ((uint16_t)packet[22] << 3);
  channels[15] &= 0x07FF;

  SBUS_StoreChannels(channels);
  return 1;
}

/**
  * @brief  SBUS 通道数据存入 (范围 0~2047 原始值)
  *         CH1->rx, CH2->ry, CH3->ly, CH4->lx
  * @param  channels：长度至少 16 的 SBUS 通道数组，每个元素低 11 位有效。
  * @retval 无。
  * @note   本工程只使用 CH1~CH4 摇杆和 CH6 开关/按键，其余通道暂未保存。
  */
static void SBUS_StoreChannels(uint16_t *channels)
{
  ps2_data.rx = channels[0];   /* CH1 -> 右摇杆 X */
  ps2_data.ry = channels[1];   /* CH2 -> 右摇杆 Y */
  ps2_data.ly = channels[2];   /* CH3 -> 左摇杆 Y */
  ps2_data.lx = channels[3];   /* CH4 -> 左摇杆 X */
  ps2_data.ch6 = channels[5];  /* CH6 -> O 键等 */
  ps2_data.connected = 1;
}

/**
  * @brief  解析单个 SBUS 字节 (由 UART 接收回调调用)
  * @param  byte：USART2 中断接收到的 1 字节数据。
  * @retval 无。
  * @note   状态转移：
  *         WAIT_HEADER 收到 0x0F -> RECEIVING；
  *         RECEIVING 收满 25 字节 -> 检查帧尾 -> 解析 -> 回到 WAIT_HEADER。
  *         WARNING：该函数在 UART 回调中执行，应保持轻量，避免阻塞。
  */
void PS2_Receiver_ParseByte(uint8_t byte)
{
  switch (sbus_state) {
    case SBUS_STATE_WAIT_HEADER:
      if (byte == SBUS_HEADER) {
        sbus_rx_buffer[0] = byte;
        sbus_rx_index = 1;
        sbus_state = SBUS_STATE_RECEIVING;
      }
      break;

    case SBUS_STATE_RECEIVING:
      sbus_rx_buffer[sbus_rx_index++] = byte;

      if (sbus_rx_index >= SBUS_PACKET_LENGTH) {
        if (sbus_rx_buffer[24] == SBUS_FOOTER) {
          SBUS_Parse_Packet(sbus_rx_buffer);
        }
        sbus_state = SBUS_STATE_WAIT_HEADER;
        sbus_rx_index = 0;
      }
      break;

    default:
      sbus_state = SBUS_STATE_WAIT_HEADER;
      sbus_rx_index = 0;
      break;
  }
}

/* UART 接收中断单字节缓冲。
 * HAL_UART_Receive_IT() 每次接收 1 字节，完成后在回调中重新启动下一次接收。
 */
static uint8_t sbus_uart_rx_byte;

/**
  * @brief  初始化 PS2/SBUS 接收模块。
  * @param  无。
  * @retval 无。
  * @note   将摇杆值初始化为 SBUS 中位 1024，状态机回到等待帧头，并启动 USART2
  *         单字节中断接收。调用前必须已执行 MX_USART2_UART_Init()。
  */
void PS2_Receiver_Init(void)
{
  /* 初始化中位值 (SBUS 中心 1024) */
  ps2_data.lx        = 1024;
  ps2_data.ly        = 1024;
  ps2_data.rx        = 1024;
  ps2_data.ry        = 1024;
  ps2_data.ch6       = 1024;
  ps2_data.buttons   = 0;
  ps2_data.connected = 1;   /* 初始化已连接 */

  sbus_rx_index = 0;
  sbus_state    = SBUS_STATE_WAIT_HEADER;

  /* 启动 USART2 接收中断 (SBUS) */
  HAL_UART_Receive_IT(&huart2, &sbus_uart_rx_byte, 1);
}

/**
  * @brief  UART 接收完成回调，解析 SBUS 数据
  * @param  huart：触发回调的 UART 句柄。
  * @retval 无。
  * @note   中断来源：USART2_IRQn -> HAL_UART_IRQHandler(&huart2) -> 本回调。
  *         触发条件：USART2 接收到 1 字节并完成 HAL_UART_Receive_IT()。
  *         涉及寄存器由 HAL 管理，包括 USART_SR/RDR/CR1 RXNEIE 等。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    PS2_Receiver_ParseByte(sbus_uart_rx_byte);
    HAL_UART_Receive_IT(&huart2, &sbus_uart_rx_byte, 1);
  }
}

/**
  * @brief  获取最近一次解析到的遥控器数据。
  * @param  data：输出结构体指针，不能为空。
  * @retval 无。
  * @note   当前实现逐字段复制，未关中断；16 位字段在 Cortex-M4 上通常可原子访问，
  *         但若未来增加多字节复合状态，建议加入临界段。
  */
void PS2_Receiver_GetData(PS2_Data_TypeDef *data)
{
  if (data == NULL) return;

  data->lx        = ps2_data.lx;
  data->ly        = ps2_data.ly;
  data->rx        = ps2_data.rx;
  data->ry        = ps2_data.ry;
  data->ch6       = ps2_data.ch6;
  data->buttons   = ps2_data.buttons;
  data->connected = ps2_data.connected;
}
