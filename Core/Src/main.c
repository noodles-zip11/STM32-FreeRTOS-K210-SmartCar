/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"
#include "pid.h"
#include "string.h"
#include "HC_SR04.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "settings.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 串口3命令接收开关：调试阶段可临时关闭，便于隔离串口干扰问题。 */
#define UART3_CMD_ENABLE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t g_ucusrtrecivedate;
extern uint8_t  RxBuffer;
extern volatile TickType_t g_last_cmd_tick;
extern volatile uint8_t g_ucMode;

/*
 * UART2 视觉帧解析状态（文件作用域）：
 * 放在这里而不是回调局部变量，是为了在 ErrorCallback 中也能复位状态机。
 * 状态定义：
 * 0=等待 0xFF；1=等待 0xFE；2=接收 4 字节数据；3=等待校验和
 */
static uint8_t s_vision_rx_state = 0;
static uint8_t s_vision_rx_data_cnt = 0;
static uint8_t s_vision_rx_data_buf[4] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * 超声波读数做“有效性判断 + 一阶低通”：
 * 1) 先过滤掉明显异常值（<=0 或过大）；
 * 2) 用上一次有效值兜底，避免上层逻辑突然拿到 0；
 * 3) 正常值用低通减小跳变，让避障/跟随更稳定。
 */
float Get_Distance_Filtered(void)
{
  static float last_valid = 30.0f;
  float distance = HC_SR04_Read();
  if ((distance <= 0.0f) || (distance > 300.0f))
  {
    osDelay(1);
    return last_valid;
  }
  last_valid = (0.7f * last_valid) + (0.3f * distance);
  osDelay(1);
  return last_valid;
}

int __io_putchar(int ch)
{
  /* printf 重定向到串口1（阻塞式，适合低频调试输出）。 */
  uint8_t c = (uint8_t)ch;
  (void)HAL_UART_Transmit(&huart1, &c, 1, 10);
  return ch;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /*
   * 初始化顺序说明（按依赖关系排列）：
   * 1) 先起外设（PWM/编码器/串口/看门狗）
   * 2) 再初始化 PID 和 Flash 参数
   * 3) 再初始化 IMU 等传感器
   * 4) 最后创建并启动 FreeRTOS 任务
   * 这样任务启动后能立即使用已准备好的资源。
   */

  // --- 1. 电机与编码器启动 ---
  /* 先启动 PWM/编码器，后续控制任务启动后才能马上闭环控制，不用再等外设准备。 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // --- 2. 串口与通信启动 ---
  /*
   * 串口接收顺序说明：
   * - UART3 命令口现在就开启，尽早响应蓝牙/遥控；
   * - UART2 视觉口放到 VisionTask 中开启，因为它依赖消息队列已经创建完成。
   */
  // USART1 仅用于调试发送，不开 RX 中断，避免悬空串口导致中断风暴。
#if UART3_CMD_ENABLE
  HAL_UART_Receive_IT(&huart3, &g_ucusrtrecivedate, 1);
#endif
  /* UART2 视觉接收放到 VisionTask 中启动，确保队列/任务已经就绪。 */

  // --- 3. 算法参数初始化 ---
  /* PID 先给默认值，再从 Flash 覆盖成上次保存值，启动状态更可控。 */
  PID_init();
  settings_load();
  g_ucMode = 0;

  // --- 4. 传感器初始化 (去掉打印，只做事) ---
  /* IMU 初始化前延时一小段时间，给上电和传感器内部稳态一点缓冲。 */
  HAL_Delay(500); // 等待上电稳定
  MPU_Init();     // 初始化MPU6050
  for (uint8_t imu_retry = 0; imu_retry < 3; imu_retry++)
  {
    if (mpu_dmp_init() == 0)
    {
      break;
    }
    HAL_Delay(100);
  }

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
extern osMessageQId CommandQueueHandle;
extern osMessageQId VisionQueueHandle;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /*
   * 这个回调同时处理两个串口：
   * - UART3：单字节命令（蓝牙/遥控）
   * - UART2：视觉模块数据帧（按协议拆包）
   * 中断里只做快速动作（入队、推进状态机、重挂接收），
   * 复杂计算留给任务上下文，避免阻塞其它中断。
   */

  // === 1. 蓝牙/遥控处理 (UART3) ===
  if (huart == &huart3)
  {
#if UART3_CMD_ENABLE
    uint8_t cmd = g_ucusrtrecivedate;
    // 仅允许约定的控制键，屏蔽随机串口噪声/乱码。
    if (CommandQueueHandle != NULL &&
        (cmd == 'A' || cmd == 'B' || cmd == 'C' || cmd == 'D' ||
         cmd == 'E' || cmd == 'F' || cmd == 'G' || cmd == 'H' ||
         cmd == 'I' || cmd == 'J' || cmd == 'K'))
    {
      xQueueSendFromISR(CommandQueueHandle, &g_ucusrtrecivedate, &xHigherPriorityTaskWoken);
    }

    // 继续接收
    HAL_UART_Receive_IT(&huart3, &g_ucusrtrecivedate, 1);
 #endif
  }
  // === 2. 视觉数据处理 (UART2) ===
  else if (huart->Instance == USART2)
  {
    uint8_t res = RxBuffer;

    /*
     * 视觉帧协议（当前实现）：
     *   帧头 0xFF 0xFE + 4字节数据(x,y) + 1字节校验和
     * 使用逐字节状态机的原因：
     * - 内存占用小，不需要大缓存；
     * - 丢字节后可以靠帧头快速重新同步。
     */
    if (s_vision_rx_state == 0U) {
      if (res == 0xFFU) {
        s_vision_rx_state = 1U;
      }
    }
    else if (s_vision_rx_state == 1U) {
      if (res == 0xFEU) {
        s_vision_rx_state = 2U;
      } else if (res == 0xFFU) {
        /* 连续收到 0xFF 时保持 state=1，能更快重新同步帧头。 */
        s_vision_rx_state = 1U;
      } else {
        s_vision_rx_state = 0U;
      }
    }
    else if (s_vision_rx_state == 2U) {
      s_vision_rx_data_buf[s_vision_rx_data_cnt++] = res;
      if (s_vision_rx_data_cnt >= 4U) {
        s_vision_rx_state = 3U;
        s_vision_rx_data_cnt = 0U;
      }
    }
    else if (s_vision_rx_state == 3U) {
      uint8_t calc_sum = (s_vision_rx_data_buf[0] + s_vision_rx_data_buf[1] +
                          s_vision_rx_data_buf[2] + s_vision_rx_data_buf[3]) & 0xFF;

      if (res == calc_sum) {
        VisionData_t data;
        /* 校验通过后再覆盖队列，保证逻辑任务拿到的是“最新有效帧”。 */
        data.x = (int16_t)((s_vision_rx_data_buf[0] << 8) | s_vision_rx_data_buf[1]);
        data.y = (int16_t)((s_vision_rx_data_buf[2] << 8) | s_vision_rx_data_buf[3]);
        if (VisionQueueHandle != NULL) {
          xQueueOverwriteFromISR(VisionQueueHandle, &data, &xHigherPriorityTaskWoken);
        }
      }
      s_vision_rx_state = 0U;
    }

    HAL_UART_Receive_IT(&huart2, &RxBuffer, 1);
  }
  /*
   * 若 ISR 中唤醒了更高优先级任务，立即请求一次上下文切换。
   * 这样命令和视觉数据能尽快进入任务处理链，降低控制延迟。
   */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /*
     * 视觉串口出错后最常见问题是“状态机卡在半帧”。
     * 处理顺序：清错误标志 -> 清解析状态 -> 重新挂接收中断。
     */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    s_vision_rx_state = 0U;
    s_vision_rx_data_cnt = 0U;
    (void)HAL_UART_Receive_IT(&huart2, &RxBuffer, 1);
  }
  else if (huart->Instance == USART3)
  {
    /* 命令串口出错后也要重挂接收，否则后续会表现为“静默失联”。 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
#if UART3_CMD_ENABLE
    (void)HAL_UART_Receive_IT(&huart3, &g_ucusrtrecivedate, 1);
#endif
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    /* TIM3 被配置为 HAL 时间基准，用于驱动 HAL_GetTick()/HAL_Delay()。 */
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
