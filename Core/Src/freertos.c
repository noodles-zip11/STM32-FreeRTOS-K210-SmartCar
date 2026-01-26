/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
double p = 0.0;
double i = 0.0;
double d = 0.0;
double a = 0.0;
short Encoder1Count =0 ;
short Encoder2Count =0 ;
float Motor1Speed = 0.00;
float Motor2Speed = 0.00;
uint16_t TimerCount=0;
int Motor1pwm;
int Motor2pwm;
extern tpid pidMotor1Speed;
extern tpid pidMotor2Speed;

extern tpid k210motion;
extern  uint8_t Usart1_ReadBuf[256];
extern  uint8_t Usart1_ReadCount;
float mile =0.00 ;

char OledString[50];
char Usart3String[50];



float g_sr04_read = 0.0;
float g_follow_pid_out=0.0;
float dist;

extern tpid pidFollow;
extern tpid mpu6050Movement;
extern tpid pid_pidHW_Tracking;
uint8_t g_read[35];
int8_t g_thisstate = 0;
int8_t g_laststate = 0;
float g_pid_out = 0.0;
float g_pid_out1 = 0.0;
float g_pid_out2 = 0.0;
uint8_t g_ucusrtrecivedate;
volatile uint8_t g_ucMode = 0 ;

float  g_fMPU6050YawMovePidOut = 0.00f;
float  g_fMPU6050YawMovePidOut1 = 0.00f;
float  g_fMPU6050YawMovePidOut2 = 0.00f;

float  k210PidOut = 0.00f;
float  k210PidOut1 = 0.00f;
float  k210PidOut2 = 0.00f;

float pitch,roll,yaw;

uint8_t  RxBuffer;          // 临时存放串口收到的这1个字节
int16_t Vision_x = 0;      // 解析出来的 X 坐标 (给逻辑代码用)
int16_t Vision_y = 0;      // 解析出来的 Y 坐标 (给逻辑代码用)
uint8_t Vision_Status = 0; // 状态标记：1表示刚刚更新了数据

// --- 状态机相关变量 ---
uint8_t rx_state = 0;      // 协议解析状态
uint8_t rx_data_buf[4];    // 临时存放 X高, X低, Y高, Y低
uint8_t rx_data_cnt = 0;   // 数据计数器
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ControlTaskHandle;
osThreadId SensorTaskHandle;
osThreadId VisionTaskHandle;
osThreadId LogicTaskHandle;
osThreadId StartUITaskHandle;
osMessageQId CommandQueueHandle;
osMessageQId VisionQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControlTask(void const * argument);
void StartSensorTask(void const * argument);
void StartVisionTask(void const * argument);
void StartLogicTask(void const * argument);
void StartTask06(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of CommandQueue */
  osMessageQDef(CommandQueue, 10, uint8_t);
  CommandQueueHandle = osMessageCreate(osMessageQ(CommandQueue), NULL);

  /* definition and creation of VisionQueue */
  osMessageQDef(VisionQueue, 5, uint32_t);
  VisionQueueHandle = osMessageCreate(osMessageQ(VisionQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityHigh, 0, 512);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensorTask, osPriorityAboveNormal, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of VisionTask */
  osThreadDef(VisionTask, StartVisionTask, osPriorityNormal, 0, 128);
  VisionTaskHandle = osThreadCreate(osThread(VisionTask), NULL);

  /* definition and creation of LogicTask */
  osThreadDef(LogicTask, StartLogicTask, osPriorityNormal, 0, 128);
  LogicTaskHandle = osThreadCreate(osThread(LogicTask), NULL);

  /* definition and creation of StartUITask */
  osThreadDef(StartUITask, StartTask06, osPriorityIdle, 0, 128);
  StartUITaskHandle = osThreadCreate(osThread(StartUITask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // 开启编码器
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  /* Infinite loop */
  for(;;)
  {
    Encoder1Count=(short)__HAL_TIM_GET_COUNTER(&htim4);
    Encoder2Count=(short)__HAL_TIM_GET_COUNTER(&htim2);

    __HAL_TIM_SET_COUNTER(&htim4,0);
    __HAL_TIM_SET_COUNTER(&htim2,0);

    Motor1Speed = (float)Encoder1Count * 100 /9.6/11/4;
    Motor2Speed = -(float)Encoder2Count * 100 /9.6/11/4;

    mile+= 0.02*Motor1Speed*22;

    Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));


    osDelay(10);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
  for(;;)
  {
    g_read[0] = READ_HW_OUT_1;
    g_read[1] = READ_HW_OUT_2;
    g_read[2] = READ_HW_OUT_3;
    g_read[3] = READ_HW_OUT_4;

    dist = Get_Distance_Filtered();

    g_sr04_read = HC_SR04_Read();

    mpu_dmp_get_data(&pitch,&roll,&yaw);

    osDelay(10);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartVisionTask */
/**
* @brief Function implementing the VisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVisionTask */
void StartVisionTask(void const * argument)
{
  /* USER CODE BEGIN StartVisionTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END StartVisionTask */
}

/* USER CODE BEGIN Header_StartLogicTask */
/**
* @brief Function implementing the LogicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogicTask */
void StartLogicTask(void const * argument)
{
  /* USER CODE BEGIN StartLogicTask */
  uint8_t cmd_char;
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(CommandQueueHandle, &cmd_char, 0) == pdTRUE)
    {
      // 收到指令，开始干活
      switch(cmd_char)
      {
        case 'A': motorPidSetSpeed(2,2); break;
        case 'B': motorPidSetSpeed(-2,-2); break;
        case 'C': motorPidSetSpeed(2,1); break;
        case 'D': motorPidSetSpeed(1,2); break;
        case 'E': motorPidSetSpeed(0,0); break;
        case 'F': motorSpeedUp(); break;
        case 'G': motorSpeedCut(); break;
        case 'J':
          g_ucMode++;
          if(g_ucMode > 5) g_ucMode = 1;
          break;
        case 'K': g_ucMode = 0; break;
        case 'H': // 转向指令可以直接改全局目标值
          mpu6050Movement.targer_val += 90;
          break;
        case 'I':
          mpu6050Movement.targer_val -= 90;
      }
    }
    switch (g_ucMode) {

      case 1:
        if (g_read[0] == 0 && g_read[1] == 0 && g_read[2] == 0 && g_read[3] == 0)
        {
          //motorPidSetSpeed(1,1);
          g_thisstate = 0;
        }

        else if (g_read[0] == 0  && g_read[1] == 0 && g_read[2] == 1 && g_read[3] == 0)
        {
          //motorPidSetSpeed(2,1);

          g_thisstate = 1;
        }
        else if (g_read[0] == 0  && g_read[1] == 1 && g_read[2] == 0 && g_read[3] == 0)
        {
          //motorPidSetSpeed(2,1);

          g_thisstate = -1;
        }
        else if (g_read[0] == 0  && g_read[1] == 0 && g_read[2] == 0 && g_read[3] == 1)
        {
          //motorPidSetSpeed(3,1);
          g_thisstate = 2;
        }

        else if (g_read[0] == 0  && g_read[1] == 0 && g_read[2] == 1 && g_read[3] == 1)
        {
          //motorPidSetSpeed(3,1);
          g_thisstate = 3;
        }
        else if (g_read[0] == 1 && g_read[1] == 0 &&  g_read[2] == 0 && g_read[3] == 0)
        {
          //motorPidSetSpeed(3,1);
          g_thisstate = -2;
        }

        else if (g_read[0] == 1  && g_read[1] == 1 && g_read[2] == 0 && g_read[3] == 0)
        {
          //motorPidSetSpeed(3,1);
          g_thisstate = -3;
        }

        g_pid_out = PID_realize(&pid_pidHW_Tracking,g_thisstate);

        g_pid_out1 = 3 + g_pid_out;
        g_pid_out2 = 3 - g_pid_out;

        if (g_pid_out1>5) g_pid_out1 = 5;
        if (g_pid_out2>5) g_pid_out2 = 5;
        if (g_pid_out1<0) g_pid_out1 = 1;
        if (g_pid_out2<0) g_pid_out2 = 1;

        motorPidSetSpeed(g_pid_out1, g_pid_out2);

        g_laststate = g_thisstate ;
        break;


      case 2:
        if(dist > 20) {
          motorPidSetSpeed(2, 2);
        } else {
          motorPidSetSpeed(0, 0);   osDelay(100);  // 停车
          motorPidSetSpeed(-1.5, -1.5); osDelay(300); // 后退
          motorPidSetSpeed(2, -2);  osDelay(400);  // 右转
          motorPidSetSpeed(0, 0);   osDelay(200);  // 停一下观察
        }
        break;


      case 3:
        if(g_sr04_read < 60 )
        {
          g_follow_pid_out = PI_realize(&pidFollow, g_sr04_read);


          if (g_follow_pid_out > 5) {
            g_follow_pid_out =5 ;
          }
          if (g_follow_pid_out<-5) {
            g_follow_pid_out = -5 ;
          }
          motorPidSetSpeed(g_follow_pid_out,g_follow_pid_out);
        }
        else {
          motorPidSetSpeed(0,0);
        }
        break;


      case 4:
        while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}

        g_fMPU6050YawMovePidOut = PID_realize(&mpu6050Movement,yaw);

        g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;

        g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;

        if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;
        if(g_fMPU6050YawMovePidOut1 < 0 ) g_fMPU6050YawMovePidOut1 =0;
        if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
        if(g_fMPU6050YawMovePidOut2 < 0) g_fMPU6050YawMovePidOut2 = 0;

        motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
        break;
    }
    osDelay(50);
  }
  /* USER CODE END StartLogicTask */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the StartUITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  OLED_Init();
  OLED_Clear();
  char oled_buf[32]; // 局部缓冲区
  /* Infinite loop */
  for(;;)
  {
  // =========================================================
    //  第 0 行：显示 模式 和 电池电压
    //  格式： "M:1  Bat:7.4V"
    // =========================================================
    // ADC读取放在这里或者SensorTask都可以，假设比较快直接读
    sprintf(oled_buf, "M:%d Bat:%.1fV", g_ucMode, adcGetBatteryVoltage());
    OLED_ShowString(0, 0, (uint8_t *)oled_buf, 12);


    // =========================================================
    //  第 1 行：显示 左右电机目标速度
    //  格式： "L:100 R:100" (预留空格防止数字变短后有残留)
    // =========================================================
    // %.0f 表示不显示小数，节省空间；%-4.0f 表示左对齐占4位
    sprintf(oled_buf, "L:%-4.0f R:%-4.0f", Motor1Speed, Motor2Speed);
    OLED_ShowString(0, 1, (uint8_t *)oled_buf, 12);


    // =========================================================
    //  第 2 行：显示 超声波距离 和 里程
    //  格式： "D:120cm M:1.2"
    // =========================================================
    // g_dist 是 SensorTask 更新的全局变量
    sprintf(oled_buf, "D:%-3.0fcm M:%.1f", dist, mile);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 12);


    // =========================================================
    //  第 3 行：动态区域 (根据模式显示特有信息)
    // =========================================================
    if(g_ucMode == 5)
    {
        // 如果是 K210 模式，显示视觉坐标
        sprintf(oled_buf, "K210 X: %d   ", Vision_x); //后面加空格是为了覆盖掉旧字
    }
    else if(g_ucMode == 4)
    {
        // 如果是 6050 走直线模式，显示 Yaw 角度
        sprintf(oled_buf, "Yaw: %-5.1f   ", yaw);
    }
    else
    {
        // 其他模式（如循迹/避障），可以显示状态或者是空的
        sprintf(oled_buf, "Running...    ");
    }
    OLED_ShowString(0, 3, (uint8_t *)oled_buf, 12);


    // =========================================================
    //  串口发送 (可选，不想刷屏可以把这里注释掉)
    // =========================================================
    // sprintf(oled_buf, "Mode:%d Dist:%.1f\r\n", g_ucMode, g_dist);
    // HAL_UART_Transmit(&huart3, (uint8_t *)oled_buf, strlen(oled_buf), 10);

    // 刷新频率：每秒 5 次，既流畅又不卡机
    osDelay(200);
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

