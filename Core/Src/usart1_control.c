/**
 ******************************************************************************
 * @file    usart1_control.c
 * @brief   上位机串口控制实现 (USART1 + DMA+IDLE)
 *
 * 协议摘要:
 *   帧头(0xDF)
 *   目标地址 (U8)
 *   本机地址 (U8)
 *   类型 A (U8)
 *   类型 B (U8)
 *   数据长度 (U8) = 0x06
 *   数据区 6 字节:
 *     [0..1] S16 小端: X 方向速度 (右为正, mm/s)
 *     [2..3] S16 小端: Y 方向速度 (前为正, mm/s)
 *     [4..5] S16 小端: 旋转速度 (顺时针为正, mm/s)
 *   帧尾(0xFD)
 *   校验 (U8): 从帧头到帧尾(含)累加和取低 8 位
 *
 * 本模块职责:
 *   - 通过 HAL_UARTEx_ReceiveToIdle_DMA 启动 USART1 的 DMA+IDLE 接收
 *   - 在 HAL_UARTEx_RxEventCallback 中按协议解析数据
 *   - 将解析出的 vx/vy/w (mm/s) 填入 Uart1_ControlCmd_t 供上层查询
 *   - 收到合法指令时回传 0x43 作为应答
 *
 * 匀速位移 (0x64):
 *   数据区 14 字节:
 *     [0..3]   S32 小端: X 位移 (厘米 × 100)
 *     [4..7]   S32 小端: Y 位移 (厘米 × 100)
 *     [8..11]  S32 小端: Z 位移/航向 (厘米 × 100)
 *     [12..13] U16 小端: 最大合速度 (厘米/秒 × 100)
 *   单位转换: raw_value / 10.0 → mm 或 mm/s
 *   当前只处理 0x64 匀速平移
 *
 * 测试指令 (波特率 115200, 十六进制发送):
 *
 *   [0x67] 前进 300 mm/s:   DF 01 97 02 67 06 00 00 2C 01 00 00 FD 10
 *   [0x67] 后退 300 mm/s:   DF 01 97 02 67 06 00 00 D4 FE 00 00 FD B5
 *   [0x67] 右直行 300 mm/s:  DF 01 97 02 67 06 2C 01 00 00 00 00 FD 10
 *   [0x67] 左直行 300 mm/s:  DF 01 97 02 67 06 D4 FE 00 00 00 00 FD B5
 *   [0x67] 顺时针旋转 300:   DF 01 97 02 67 06 00 00 00 00 2C 01 FD 10
 *   [0x67] 停止:            DF 01 97 02 67 06 00 00 00 00 00 00 FD E3
 *
 *   [0x64] 匀速位移 X+20cm (200mm), 速度 50cm/s:
 *     DF 01 97 02 64 0E D0 07 00 00 00 00 00 00 00 00 00 00 88 13 FD ??
 *
 *   解析成功时单片机回传 0x43
 ******************************************************************************
 */

#include "usart1_control.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>

/* 头文件说明：
 * usart1_control.h : 定义上位机控制命令结构体、命令类型和模块 API。
 * usart.h          : 提供 huart1 句柄，供 DMA+IDLE 接收和应答发送使用。
 * stdint.h         : 提供 uint8_t/uint16_t/int32_t 等固定宽度整数类型。
 * string.h         : 提供 memcpy()/memmove()，用于串口流缓冲区维护。
 */

/* USART1 DMA 接收缓冲区配置。
 * UART1_RX_BUF_SIZE           : 单次 DMA+IDLE 接收缓冲区大小，单位 byte。
 * UART1_STREAM_BUF_SIZE       : 跨多次 IDLE 回调保存的流式缓冲区大小，单位 byte。
 * UART1_FRAME_WAIT_TIMEOUT_MS : 半帧等待超时，单位 ms；超过后清空残留半帧。
 */
#define UART1_RX_BUF_SIZE 128
#define UART1_STREAM_BUF_SIZE 256
#define UART1_FRAME_WAIT_TIMEOUT_MS 100U

/* 协议常量 (当前实现: 匀速速度控制 0x67, 匀速位移 0x64)。
 * 帧格式：
 * [0]    0xDF 帧头
 * [1]    目标地址，MCU=0x01
 * [2]    源地址，上位机=0x97
 * [3]    类型 A，运动控制=0x02
 * [4]    类型 B，0x67=速度控制，0x64=位移控制
 * [5]    数据长度
 * [6..]  payload
 * [N-2]  0xFD 帧尾
 * [N-1]  checksum，从帧头到帧尾逐字节累加取低 8 位
 */
#define UART1_FRAME_HEAD 0xDF
#define UART1_FRAME_TAIL 0xFD

#define UART1_ADDR_MCU 0x01
#define UART1_ADDR_PC 0x97

#define UART1_TYPE_A_MOVE 0x02
#define UART1_TYPE_B_VELOCITY 0x67     /* 持续速度控制 */
#define UART1_TYPE_B_DISPLACEMENT 0x64 /* 定点匀速位移 */

#define UART1_DATA_LEN_VELOCITY 0x06
#define UART1_DATA_LEN_DISPLACEMENT 14

#define UART1_FRAME_LEN_VELOCITY                                               \
  (1U /* head */ + 1U /* dst */ + 1U /* src */ + 2U /* typeA/typeB */ +        \
   1U /* len */ + UART1_DATA_LEN_VELOCITY /* payload */ + 1U /* tail */ +      \
   1U /* checksum */)

#define UART1_FRAME_LEN_DISPLACEMENT                                           \
  (1U /* head */ + 1U /* dst */ + 1U /* src */ + 2U /* typeA/typeB */ +        \
   1U /* len */ + UART1_DATA_LEN_DISPLACEMENT /* payload */ + 1U /* tail */ +  \
   1U /* checksum */)

/* 速度缩放：协议中 S16 的 1 LSB 对应 1 mm/s。 */
#define UART1_SPEED_SCALE_MMPS_PER_LSB 1.0f

/* 位移帧单位转换。
 * 协议值 = 厘米 * 100；目标单位需要 mm：
 * (raw / 100.0) cm * 10 = raw / 10.0 mm。
 */
#define UART1_DISP_CM_TO_MM_FACTOR 10.0f

/* DMA 接收缓冲区，只在本模块内部可见。
 * HAL DMA 写入该缓冲区；IDLE 或缓冲区事件触发后由回调解析 Size 字节。
 */
static uint8_t s_uart1_rx_buf[UART1_RX_BUF_SIZE];

/* 跨 DMA+IDLE 回调保存字节流。
 * 目的：无线串口或上位机发送过程中可能在一帧中间产生停顿，使一帧被拆成
 * 多次 IDLE 回调；流缓冲可保留前半帧，等待后续字节补齐。
 */
static uint8_t s_uart1_stream_buf[UART1_STREAM_BUF_SIZE];
static uint16_t s_uart1_stream_len = 0U;
static uint32_t s_uart1_stream_last_tick = 0U;
static uint16_t s_uart1_last_rx_size = 0U;

/* 调试信息结构体。
 * 用于记录 0x64 位移帧解析过程中的状态、长度、校验、原始字段等。
 * volatile 原因：在 UART 回调上下文写入，主循环/调试器可能异步读取。
 */
typedef struct {
  uint32_t seq;
  uint16_t rx_size;
  uint16_t stream_len;
  uint16_t frame_len;
  uint8_t code;
  uint8_t hdr[6];
  uint8_t tail;
  uint8_t checksum_calc;
  uint8_t checksum_rx;
  int32_t x_raw;
  int32_t y_raw;
  int32_t z_raw;
  uint16_t speed_raw;
} Uart1_DebugInfo_t;

#define UART1_DBG64_SEEN 1U
#define UART1_DBG64_WAIT_MORE 2U
#define UART1_DBG64_BAD_TAIL 3U
#define UART1_DBG64_BAD_SUM 4U
#define UART1_DBG64_PARSE_OK 5U
#define UART1_DBG64_PARSE_FAIL 6U

static volatile Uart1_DebugInfo_t s_uart1_debug;
static volatile uint8_t s_uart1_debug_pending = 0U;

/* 最近一次解析成功的控制命令
 * 标记为 volatile 是为了防止编译器在中断与主循环之间优化掉读写
 */
static volatile Uart1_ControlCmd_t s_latest_cmd;

/**
 * @brief  将两个小端字节组合成有符号 16 位整数。
 * @param  low：低 8 位字节。
 * @param  high：高 8 位字节。
 * @retval int16_t：按二进制补码解释后的 S16 值。
 * @note   组合公式：value = low | (high << 8)。
 */
static int16_t BytesToS16LE(uint8_t low, uint8_t high) {
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/**
 * @brief  将两个小端字节组合成无符号 16 位整数。
 * @param  low：低 8 位字节。
 * @param  high：高 8 位字节。
 * @retval uint16_t：组合后的 U16 值。
 */
static uint16_t BytesToU16LE(uint8_t low, uint8_t high) {
  return (uint16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/**
 * @brief  将四个小端字节组合成有符号 32 位整数。
 * @param  p：指向 4 字节小端数据的指针，p[0] 为最低字节。
 * @retval int32_t：按二进制补码解释后的 S32 值。
 */
static int32_t BytesToS32LE(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/**
 * @brief  计算协议帧累加校验。
 * @param  frame：指向完整帧起始地址。
 * @param  frame_len：完整帧长度，包含末尾 checksum 字节。
 * @retval uint8_t：从 frame[0] 累加到 frame[frame_len-2] 的低 8 位和。
 * @note   若 frame 为空或长度小于 2，返回 0。
 */
static uint8_t Uart1_ChecksumCalc(const uint8_t *frame, uint16_t frame_len) {
  uint8_t sum = 0U;

  if (frame == NULL || frame_len < 2U) {
    return 0U;
  }

  for (uint16_t i = 0U; i < (uint16_t)(frame_len - 1U); i++) {
    sum = (uint8_t)(sum + frame[i]);
  }

  return sum;
}

/**
 * @brief  记录 0x64 位移帧调试状态。
 * @param  code：调试状态码，取值 UART1_DBG64_*。
 * @param  frame_len：当前期待或解析的帧长。
 * @param  checksum_calc：计算得到的校验值。
 * @retval 无。
 * @note   该函数在 UART 接收回调链路中调用，只做内存记录，不发送串口。
 */
static void Uart1_Debug64Set(uint8_t code, uint16_t frame_len,
                             uint8_t checksum_calc) {
  s_uart1_debug.seq++;
  s_uart1_debug.code = code;
  s_uart1_debug.rx_size = s_uart1_last_rx_size;
  s_uart1_debug.stream_len = s_uart1_stream_len;
  s_uart1_debug.frame_len = frame_len;
  s_uart1_debug.checksum_calc = checksum_calc;

  for (uint8_t i = 0U; i < 6U; i++) {
    s_uart1_debug.hdr[i] =
        (i < s_uart1_stream_len) ? s_uart1_stream_buf[i] : 0U;
  }

  if (frame_len > 1U && s_uart1_stream_len >= frame_len) {
    s_uart1_debug.tail = s_uart1_stream_buf[frame_len - 2U];
    s_uart1_debug.checksum_rx = s_uart1_stream_buf[frame_len - 1U];
  } else {
    s_uart1_debug.tail = 0U;
    s_uart1_debug.checksum_rx = 0U;
  }

  if (s_uart1_stream_len >= (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
    s_uart1_debug.x_raw = BytesToS32LE(&s_uart1_stream_buf[6]);
    s_uart1_debug.y_raw = BytesToS32LE(&s_uart1_stream_buf[10]);
    s_uart1_debug.z_raw = BytesToS32LE(&s_uart1_stream_buf[14]);
    s_uart1_debug.speed_raw =
        BytesToU16LE(s_uart1_stream_buf[18], s_uart1_stream_buf[19]);
  } else {
    s_uart1_debug.x_raw = 0;
    s_uart1_debug.y_raw = 0;
    s_uart1_debug.z_raw = 0;
    s_uart1_debug.speed_raw = 0U;
  }

  s_uart1_debug_pending = 1U;
}

/**
 * @brief  尝试解析 0x67 持续速度控制帧。
 * @param  buf：指向候选完整帧的缓冲区。
 * @param  size：候选帧长度，必须等于 UART1_FRAME_LEN_VELOCITY。
 * @retval 1：解析成功并更新 s_latest_cmd。
 * @retval 0：长度、地址、类型、帧尾或校验错误。
 * @note   payload 格式：
 *         [6..7]   S16 LE vx，单位 mm/s；
 *         [8..9]   S16 LE vy，单位 mm/s；
 *         [10..11] S16 LE w，单位 mm/s。
 */
static uint8_t ParseVelocityFrame(const uint8_t *buf, uint16_t size) {
  if (size != (uint16_t)UART1_FRAME_LEN_VELOCITY)
    return 0U;
  if (buf[0] != UART1_FRAME_HEAD || buf[12] != UART1_FRAME_TAIL)
    return 0U;
  if (buf[1] != UART1_ADDR_MCU || buf[2] != UART1_ADDR_PC)
    return 0U;
  if (buf[3] != UART1_TYPE_A_MOVE || buf[4] != UART1_TYPE_B_VELOCITY)
    return 0U;
  if (buf[5] != UART1_DATA_LEN_VELOCITY)
    return 0U;

  uint8_t sum = 0U;
  for (uint8_t i = 0; i <= 12; i++)
    sum = (uint8_t)(sum + buf[i]);
  if (sum != buf[13])
    return 0U;

  Uart1_ControlCmd_t cmd = {0};
  cmd.type = UART1_CMD_VELOCITY;
  cmd.vx_mmps =
      (float)BytesToS16LE(buf[6], buf[7]) * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.vy_mmps =
      (float)BytesToS16LE(buf[8], buf[9]) * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.w_mmps =
      (float)BytesToS16LE(buf[10], buf[11]) * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.valid = 1U;

  s_latest_cmd = cmd;
  return 1U;
}

/**
 * @brief  尝试解析 0x64 定点匀速位移控制帧。
 * @param  buf：指向候选完整帧的缓冲区。
 * @param  size：候选帧长度，必须等于 UART1_FRAME_LEN_DISPLACEMENT。
 * @retval 1：解析成功并更新 s_latest_cmd。
 * @retval 0：长度、地址、类型、帧尾或校验错误。
 * @note   payload 格式：
 *         [6..9]   S32 LE X 位移，协议单位 cm*100，转换为 mm；
 *         [10..13] S32 LE Y 位移，协议单位 cm*100，转换为 mm；
 *         [14..17] S32 LE Z/航向字段，当前仅保存为 target_z_mm；
 *         [18..19] U16 LE 速度，协议单位 cm/s*100，转换为 mm/s。
 */
static uint8_t ParseDisplacementFrame(const uint8_t *buf, uint16_t size) {
  if (size != (uint16_t)UART1_FRAME_LEN_DISPLACEMENT)
    return 0U;
  if (buf[0] != UART1_FRAME_HEAD || buf[20] != UART1_FRAME_TAIL)
    return 0U;
  if (buf[1] != UART1_ADDR_MCU || buf[2] != UART1_ADDR_PC)
    return 0U;
  if (buf[3] != UART1_TYPE_A_MOVE || buf[4] != UART1_TYPE_B_DISPLACEMENT)
    return 0U;
  if (buf[5] != UART1_DATA_LEN_DISPLACEMENT)
    return 0U;

  uint8_t sum = 0U;
  for (uint8_t i = 0; i <= 20; i++)
    sum = (uint8_t)(sum + buf[i]);
  if (sum != buf[21])
    return 0U;

  Uart1_ControlCmd_t cmd = {0};
  cmd.type = UART1_CMD_DISPLACEMENT;
  /*
   * 协议: F32 表示 厘米 × 100 (4字节小端整数), 目标单位 mm
   * 转换: (raw / 100.0) cm × 10 = raw / 10.0 mm
   * 例如: 0.2m = 20cm -> 协议值 2000 -> 2000/10 = 200mm ✓
   */
  cmd.target_x_mm = (float)BytesToS32LE(&buf[6]) / UART1_DISP_CM_TO_MM_FACTOR;
  cmd.target_y_mm = (float)BytesToS32LE(&buf[10]) / UART1_DISP_CM_TO_MM_FACTOR;
  cmd.target_z_mm = (float)BytesToS32LE(&buf[14]) / UART1_DISP_CM_TO_MM_FACTOR;

  /* F16 速度: 厘米/秒 × 100, 转 mm/s => raw / 10.0 */
  cmd.target_speed_mmps = (float)BytesToU16LE(buf[18], buf[19]) / UART1_DISP_CM_TO_MM_FACTOR;
  cmd.valid = 1U;

  s_latest_cmd = cmd;
  return 1U;
}

/**
 * @brief  在单次 DMA 数据块中扫描并解析 0x64 位移帧。
 * @param  data：本次 DMA+IDLE 回调收到的数据块。
 * @param  size：数据块长度，单位 byte。
 * @retval 1：找到并解析成功。
 * @retval 0：未找到完整合法位移帧。
 * @note   该函数允许数据块前面存在噪声字节；主要用于调试/兼容一次收完整帧的情况。
 */
static uint8_t ParseRawDisplacementBlock(const uint8_t *data, uint16_t size) {
  if (data == NULL || size < (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
    return 0U;
  }

  for (uint16_t i = 0U;
       i <= (uint16_t)(size - (uint16_t)UART1_FRAME_LEN_DISPLACEMENT); i++) {
    if (data[i] == UART1_FRAME_HEAD &&
        data[i + 4U] == UART1_TYPE_B_DISPLACEMENT &&
        data[i + 5U] == UART1_DATA_LEN_DISPLACEMENT &&
        ParseDisplacementFrame(&data[i],
                               (uint16_t)UART1_FRAME_LEN_DISPLACEMENT)) {
      return 1U;
    }
  }

  return 0U;
}

/**
 * @brief  从流缓冲区头部丢弃指定数量字节。
 * @param  count：要丢弃的字节数。
 * @retval 无。
 * @note   使用 memmove() 将剩余字节前移；count 大于等于当前长度时直接清空。
 */
static void Uart1_StreamDrop(uint16_t count) {
  if (count >= s_uart1_stream_len) {
    s_uart1_stream_len = 0U;
    return;
  }

  memmove(s_uart1_stream_buf, &s_uart1_stream_buf[count],
          (size_t)(s_uart1_stream_len - count));
  s_uart1_stream_len = (uint16_t)(s_uart1_stream_len - count);
}

/**
 * @brief  检查半帧等待是否超时，超时则清空流缓冲。
 * @param  now_tick：当前 HAL_GetTick()，单位 ms。
 * @retval 无。
 * @note   用无符号减法处理 HAL tick 回绕。超时阈值为 UART1_FRAME_WAIT_TIMEOUT_MS。
 */
static void Uart1_StreamResetIfTimeout(uint32_t now_tick) {
  if (s_uart1_stream_len > 0U &&
      (uint32_t)(now_tick - s_uart1_stream_last_tick) >
          UART1_FRAME_WAIT_TIMEOUT_MS) {
    s_uart1_stream_len = 0U;
  }
}

/**
 * @brief  将新收到的数据追加到流缓冲区。
 * @param  data：新数据指针。
 * @param  size：新数据长度，单位 byte。
 * @retval 无。
 * @note   若缓冲区空间不足，会丢弃最早的字节，保留最近数据以提高重新同步概率。
 */
static void Uart1_StreamPush(const uint8_t *data, uint16_t size) {
  if (data == NULL || size == 0U) {
    return;
  }

  if (size >= UART1_STREAM_BUF_SIZE) {
    data = &data[size - UART1_STREAM_BUF_SIZE];
    size = UART1_STREAM_BUF_SIZE;
    s_uart1_stream_len = 0U;
  } else if ((uint16_t)(s_uart1_stream_len + size) > UART1_STREAM_BUF_SIZE) {
    Uart1_StreamDrop((uint16_t)(s_uart1_stream_len + size -
                                UART1_STREAM_BUF_SIZE));
  }

  memcpy(&s_uart1_stream_buf[s_uart1_stream_len], data, size);
  s_uart1_stream_len = (uint16_t)(s_uart1_stream_len + size);
}

/**
 * @brief  从流缓冲区中解析尽可能多的完整协议帧。
 * @param  无。
 * @retval 1：至少解析成功一帧，需要发送 ACK。
 * @retval 0：没有解析到合法完整帧。
 * @note   状态机逻辑：
 *         1. 丢弃帧头 0xDF 之前的噪声；
 *         2. 检查地址和类型 A；
 *         3. 根据类型 B 和长度字段确定期望帧长；
 *         4. 等待足够字节，校验帧尾和 checksum；
 *         5. 调用对应解析函数并丢弃已处理帧。
 */
static uint8_t Uart1_StreamParse(void) {
  uint8_t ack_success = 0U;

  while (s_uart1_stream_len > 0U) {
    uint16_t head_index = 0U;
    while (head_index < s_uart1_stream_len &&
           s_uart1_stream_buf[head_index] != UART1_FRAME_HEAD) {
      head_index++;
    }

    if (head_index > 0U) {
      Uart1_StreamDrop(head_index);
    }

    if (s_uart1_stream_len < 6U) {
      break;
    }

    if (s_uart1_stream_buf[1] != UART1_ADDR_MCU ||
        s_uart1_stream_buf[2] != UART1_ADDR_PC ||
        s_uart1_stream_buf[3] != UART1_TYPE_A_MOVE) {
      Uart1_StreamDrop(1U);
      continue;
    }

    uint16_t frame_len = 0U;

    if (s_uart1_stream_buf[4] == UART1_TYPE_B_VELOCITY &&
        s_uart1_stream_buf[5] == UART1_DATA_LEN_VELOCITY) {
      frame_len = (uint16_t)UART1_FRAME_LEN_VELOCITY;
    } else if (s_uart1_stream_buf[4] == UART1_TYPE_B_DISPLACEMENT &&
               s_uart1_stream_buf[5] == UART1_DATA_LEN_DISPLACEMENT) {
      frame_len = (uint16_t)UART1_FRAME_LEN_DISPLACEMENT;
      Uart1_Debug64Set(UART1_DBG64_SEEN, frame_len, 0U);
    } else {
      Uart1_StreamDrop(1U);
      continue;
    }

    if (s_uart1_stream_len < frame_len) {
      if (frame_len == (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
        Uart1_Debug64Set(UART1_DBG64_WAIT_MORE, frame_len, 0U);
      }
      break;
    }

    uint8_t checksum_calc = Uart1_ChecksumCalc(s_uart1_stream_buf, frame_len);
    if (s_uart1_stream_buf[frame_len - 2U] != UART1_FRAME_TAIL) {
      if (frame_len == (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
        Uart1_Debug64Set(UART1_DBG64_BAD_TAIL, frame_len, checksum_calc);
      }
      Uart1_StreamDrop(1U);
      continue;
    }

    if (checksum_calc != s_uart1_stream_buf[frame_len - 1U]) {
      if (frame_len == (uint16_t)UART1_FRAME_LEN_DISPLACEMENT) {
        Uart1_Debug64Set(UART1_DBG64_BAD_SUM, frame_len, checksum_calc);
      }
      Uart1_StreamDrop(1U);
      continue;
    }

    if (frame_len == (uint16_t)UART1_FRAME_LEN_VELOCITY) {
      if (ParseVelocityFrame(s_uart1_stream_buf, frame_len)) {
        ack_success = 1U;
      }
    } else {
      if (ParseDisplacementFrame(s_uart1_stream_buf, frame_len)) {
        Uart1_Debug64Set(UART1_DBG64_PARSE_OK, frame_len, checksum_calc);
        ack_success = 1U;
      } else {
        Uart1_Debug64Set(UART1_DBG64_PARSE_FAIL, frame_len, checksum_calc);
      }
    }

    Uart1_StreamDrop(frame_len);
  }

  return ack_success;
}

/**
 * @brief  初始化 USART1 上位机控制模块。
 * @param  无。
 * @retval 无。
 * @note   调用前必须完成 MX_USART1_UART_Init() 和 MX_DMA_Init()。
 *         初始化最近命令、流缓冲和调试状态后，启动 HAL_UARTEx_ReceiveToIdle_DMA()。
 *         关闭 DMA 半传输中断 HT，避免半包回调造成额外解析负担。
 */
void Uart1_Control_Init(void) {
  /* 初始化最近命令为无效状态 */
  s_latest_cmd.type = UART1_CMD_NONE;
  s_latest_cmd.vx_mmps = 0.0f;
  s_latest_cmd.vy_mmps = 0.0f;
  s_latest_cmd.w_mmps = 0.0f;
  s_latest_cmd.valid = 0U;
  s_uart1_stream_len = 0U;
  s_uart1_stream_last_tick = HAL_GetTick();
  s_uart1_last_rx_size = 0U;
  s_uart1_debug_pending = 0U;

  /* 启动 USART1 的 DMA+IDLE 接收
   * - huart1: 来自 usart.c 的 UART1 句柄
   * - s_uart1_rx_buf: DMA 接收缓冲区
   * - UART1_RX_BUF_SIZE: 最大接收长度
   */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  /* 关闭半传输中断，避免产生过多中断回调 */
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

/**
 * @brief  获取最近一次解析成功的上位机控制命令。
 * @param  out：输出结构体指针，不能为空。
 * @retval 无。
 * @note   复制 s_latest_cmd 时短暂关闭全局中断，避免 USART1 回调同时写入结构体。
 *         WARNING：关中断时间很短，只包含结构体拷贝，不应在此处加入耗时逻辑。
 */
void Uart1_Control_GetLatestCmd(Uart1_ControlCmd_t *out) {
  if (out == NULL) {
    return;
  }
  /* 简单的"临界区": 复制结构体时短暂关中断, 防止中断同时修改 s_latest_cmd */
  __disable_irq();
  *out = s_latest_cmd;
  __enable_irq();
}

/**
 * @brief  清除最近命令的有效标志。
 * @param  无。
 * @retval 无。
 * @note   上层处理完一次性位移命令后调用，避免重复执行同一目标。
 */
void Uart1_Control_ClearCmd(void) {
  __disable_irq();
  s_latest_cmd.valid = 0U;
  s_latest_cmd.type = UART1_CMD_NONE;
  __enable_irq();
}

/* 应答标志：中断回调设置，主循环读取并发送。
 * volatile 原因：写入发生在 UART 回调上下文，读取发生在主循环。
 */
static volatile uint8_t s_ack_pending = 0;

/**
 * @brief  UART DMA+IDLE 接收事件回调。
 * @param  huart：触发回调的 UART 句柄。
 * @param  Size：DMA 缓冲区中本次收到的数据长度，单位 byte。
 * @retval 无。
 * @note   中断来源：USART1 IDLE 或 DMA 接收事件 -> HAL_UARTEx_RxEventCallback。
 *         本工程只处理 USART1；其它 UART 直接返回。
 *         回调内先维护跨回调流缓冲，再解析完整帧，成功后只置 s_ack_pending，
 *         实际 HAL_UART_Transmit() 在主循环 Uart1_Control_SendAck() 中执行。
 *         涉及寄存器由 HAL 管理，包括 USART SR/DR/CR1 IDLEIE 和 DMA2_Stream2。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == NULL) {
    return;
  }
  if (huart->Instance != USART1) {
    return;
  }

  /* 尝试在本次数据块中寻找并解析控制帧
   * 允许前面有噪声或一次收到多帧：匹配到的帧都会更新 s_latest_cmd，最后一帧生效
   * 解析成功则设置应答标志，由主循环发送
   */
  uint32_t now_tick = HAL_GetTick();
  s_uart1_last_rx_size = Size;
  Uart1_StreamResetIfTimeout(now_tick);
  Uart1_StreamPush(s_uart1_rx_buf, Size);
  s_uart1_stream_last_tick = now_tick;
  uint8_t ack_success = ParseRawDisplacementBlock(s_uart1_rx_buf, Size);
  if (Uart1_StreamParse()) {
    ack_success = 1U;
  }
  
  /* 尝试在本次数据块中寻找并解析控制帧 */
  /* 寻找位移帧 (0x64) */
  /* 设置应答标志（主循环负责发送） */
  if (ack_success) {
    s_ack_pending = 1;
  }

  /* 重新启动 DMA+IDLE 接收 */
  /* 先停止 DMA，再重新启动，确保状态机正常 */
  HAL_UART_AbortReceive(&huart1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

/**
 * @brief  在中断外发送协议应答。
 * @param  无。
 * @retval 无。
 * @note   应在主循环中调用。若 s_ack_pending=1，则通过 USART1 发送
 *         0x43 '\r' '\n'，超时时间 10 ms。
 *         这里使用阻塞发送，但不在中断上下文执行，降低中断阻塞风险。
 */
void Uart1_Control_SendAck(void) {
  if (s_ack_pending) {
    const uint8_t ack_buf[] = {0x43U, '\r', '\n'};
    HAL_UART_Transmit(&huart1, ack_buf, sizeof(ack_buf), 10);
    s_ack_pending = 0;
  }
}

/**
 * @brief  UART 错误回调，用于恢复 USART1 DMA+IDLE 接收。
 * @param  huart：触发错误的 UART 句柄。
 * @retval 无。
 * @note   触发条件包括帧错误、噪声错误、溢出错误等 HAL UART 错误。
 *         本函数中止当前接收并重新启动 DMA+IDLE，同时关闭半传输中断。
 *         WARNING：频繁进入该回调通常表示波特率、校验位、停止位或电平转换异常。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == NULL || huart->Instance != USART1) {
    return;
  }

  HAL_UART_AbortReceive(&huart1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}
