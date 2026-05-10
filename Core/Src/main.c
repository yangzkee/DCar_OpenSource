/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : DCar_OpenSource 小车主程序入口与系统时钟配置。
 *
 * 本文件由 STM32CubeMX/Keil MDK 工程生成框架并加入用户初始化逻辑，
 * 完成 HAL、系统时钟、GPIO、DMA、TIM、USART、SPI 以及小车控制模块初始化，
 * 随后在 while(1) 中持续调用 Loop_Run() 执行分频任务调度。
 *
 * 硬件平台 : STM32F407VETx，ARM Cortex-M4F 内核
 * 开发环境 : Keil uVision / MDK-ARM，STM32CubeMX HAL 工程
 * 晶振频率 : HSE 8 MHz
 * 系统主频 : 168 MHz，AHB=168 MHz，APB1=42 MHz，APB2=84 MHz
 * 作者     : DCar_OpenSource
 * 日期     : 2026-05-10
 * 版本     : v1.0
 * 修改记录 : v1.0 增加中文工程注释，不修改代码逻辑
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
/* 头文件说明：
 * main.h  : 全局 GPIO 宏、Error_Handler() 声明、HAL 总头文件入口。
 * dma.h   : MX_DMA_Init()，配置 USART1_RX 使用的 DMA2_Stream2。
 * spi.h   : MX_SPI2_Init() 和 hspi2，用于 ICM20602 通信。
 * tim.h   : MX_TIMx_Init() 和 htim2/3/4/5/6/8，提供编码器、定时中断和 PWM。
 * usart.h : MX_UART/USART 初始化和 huart1/2/4，提供上位机、SBUS 和调试串口。
 * gpio.h  : MX_GPIO_Init()，配置 LED、IMU CS、电机方向引脚等 GPIO。
 */
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 用户模块头文件说明：
 * imu.h              : ICM20602 通信和姿态解算。
 * loop.h             : 主循环分频调度入口 Loop_Run()。
 * motor_control.h    : 四电机速度闭环控制。
 * move_control.h     : 相对位移位置控制。
 * odometry.h         : 里程计估算。
 * ps2_receiver.h     : PS2/SBUS 遥控接收。
 * remote_to_motion.h : 遥控/手动速度到四轮运动解耦。
 * uart4_debug.h      : UART4 调试输出接口。
 * usart1_control.h   : USART1 上位机协议解析。
 */
#include "imu.h"
#include "loop.h"
#include "motor_control.h"
#include "move_control.h"
#include "odometry.h"
#include "ps2_receiver.h"
#include "remote_to_motion.h"
#include "uart4_debug.h"
#include "usart1_control.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* (分频参数已迁移至 loop.c) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  应用程序入口函数。
 * @param  无。
 * @retval int：嵌入式裸机程序通常不返回；若返回，值由 C 运行库处理。
 * @note   初始化顺序：
 *         1. HAL_Init() 复位外设状态、初始化 Flash 接口和 SysTick。
 *         2. SystemClock_Config() 配置 168 MHz 系统时钟。
 *         3. 初始化 CubeMX 配置的外设。
 *         4. 初始化上层控制模块。
 *         5. 在 while(1) 中调用 Loop_Run()。
 *         WARNING：电机控制模块会启动 TIM6 中断和 TIM8 PWM，调试时应确认电机供电安全。
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
 MX_DMA_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
 MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 上位机串口控制 (USART1 DMA+IDLE) */
  Uart1_Control_Init();

  /* 电机 PID 速度闭环初始化 (编码器 + PID + PWM) */
  MotorControl_Init();

  /* PS2/SBUS 遥控器接收模块初始化 (PA3/USART2_RX)。 */
  PS2_Receiver_Init();

  /* 遥控/手动速度到运动解耦模块初始化。 */
  RemoteToMotion_Init();

  /* 里程计初始化 */
  Odometry_Init();

  /* 移动控制初始化 */
  MoveControl_Init();

  /* ICM20602 通信检查测试 */
  if (ICM20602_Check()) {
    /* 通信正常：LED0 亮（本硬件 LED 低电平有效，推测配置）。 */
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
 } else {
    /* 通信失败：LED0 灭；可在这里添加蜂鸣器、串口打印等错误提示。 */
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  }
	
//HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 任务分频调度：1 kHz IMU | 100 Hz 运动控制 | 50 Hz 调试输出。 */
    /* 详见 Core/Src/loop.c。 */
    Loop_Run();
	
  }
  /* USER CODE END 3 */
}

/**
 * @brief  配置系统时钟到 168 MHz。
 * @param  无。
 * @retval 无。
 * @note   时钟树配置：
 *         HSE=8 MHz，PLL_M=4 -> PLL 输入 2 MHz；
 *         PLL_N=168 -> VCO=336 MHz；
 *         PLL_P=2 -> SYSCLK=168 MHz；
 *         AHB=168 MHz，APB1=42 MHz，APB2=84 MHz。
 *         FLASH_LATENCY_5 对应 168 MHz 下的 Flash 等待周期。
 *         WARNING：HSE 外部晶振失效会导致 HAL_RCC_OscConfig() 失败并进入 Error_Handler()。
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  重定向 printf 的单字符输出到 UART4。
 * @param  ch：待发送字符，低 8 位有效。
 * @retval int：返回输入字符 ch，符合 C 库 putchar 约定。
 * @note   使用 HAL_UART_Transmit() 阻塞发送 1 字节，超时时间 10 ms。
 *         WARNING：不建议在高频中断中调用 printf，可能造成实时性问题。
 */
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 10);
  return ch;
}
/* USER CODE END 4 */

/**
 * @brief  HAL 或系统初始化发生错误时的统一处理函数。
 * @param  无。
 * @retval 无。
 * @note   当前实现关闭全局中断并停留在死循环，便于调试器定位错误现场。
 *         WARNING：进入该函数后电机 PWM/方向输出的实际状态取决于错误发生时刻；
 *         若要用于量产，应增加电机急停和错误指示。
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
