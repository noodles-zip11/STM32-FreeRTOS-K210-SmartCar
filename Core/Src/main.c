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
#define UART3_CMD_ENABLE 0
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

  // --- 1. 电机与编码器启动 ---
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // --- 2. 串口与通信启动 ---
  // USART1 is used for debug TX only. Keep RX interrupt disabled to avoid IRQ storm.
  HAL_UART_Receive_IT(&huart3, &g_ucusrtrecivedate, 1);
  HAL_UART_Receive_IT(&huart2, &RxBuffer, 1);

  // --- 3. 算法参数初始化 ---
  PID_init();
  settings_load();
  g_ucMode = 0;

  // --- 4. 传感器初始化 (去掉打印，只做事) ---
  HAL_Delay(500); // 等待上电稳定
  MPU_Init();     // 初始化MPU6050
  mpu_dmp_init();

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

  // === 1. 蓝牙/遥控处理 (UART3) ===
  if (huart == &huart3)
  {
    uint8_t cmd = g_ucusrtrecivedate;
    // Only allow expected control keys, shielding random serial noise.
    if (CommandQueueHandle != NULL &&
        (cmd == 'A' || cmd == 'B' || cmd == 'C' || cmd == 'D' ||
         cmd == 'E' || cmd == 'F' || cmd == 'G' || cmd == 'H' ||
         cmd == 'I' || cmd == 'J' || cmd == 'K'))
    {
#if UART3_CMD_ENABLE
      xQueueSendFromISR(CommandQueueHandle, &g_ucusrtrecivedate, &xHigherPriorityTaskWoken);
#endif
    }

    // 继续接收
    HAL_UART_Receive_IT(&huart3, &g_ucusrtrecivedate, 1);
  }
  // === 2. 视觉数据处理 (UART2) ===
  else if (huart->Instance == USART2)
  {
    uint8_t res = RxBuffer;

    // 状态机保留在中断里！因为处理单字节非常快，不需要通过队列传单个字节（太浪费资源）
    static uint8_t rx_state = 0;
    static uint8_t rx_data_cnt = 0;
    static uint8_t rx_data_buf[4];

    if(rx_state == 0) {
      if(res == 0xFF) rx_state = 1; else rx_state = 0;
    }
    else if(rx_state == 1) {
      if(res == 0xFE) rx_state = 2; else rx_state = 0;
    }
    else if(rx_state == 2) {
      rx_data_buf[rx_data_cnt++] = res;
      if(rx_data_cnt >= 4) { rx_state = 3; rx_data_cnt = 0; }
    }
    else if(rx_state == 3) {
      uint8_t calc_sum = (rx_data_buf[0] + rx_data_buf[1] + rx_data_buf[2] + rx_data_buf[3]) & 0xFF;
      if(res == calc_sum) {
        // 校验成功！打包数据
        VisionData_t data;
        data.x = (int16_t)((rx_data_buf[0] << 8) | rx_data_buf[1]);
        data.y = (int16_t)((rx_data_buf[2] << 8) | rx_data_buf[3]);

        // Keep the latest frame only; VisionQueue length is 1.
        if (VisionQueueHandle != NULL)
        {
          xQueueOverwriteFromISR(VisionQueueHandle, &data, &xHigherPriorityTaskWoken);
        }
      }
      rx_state = 0;
    }
    HAL_UART_Receive_IT(&huart2, &RxBuffer, 1);
  }

  // 如果队列唤醒了高优先级任务，进行调度
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
