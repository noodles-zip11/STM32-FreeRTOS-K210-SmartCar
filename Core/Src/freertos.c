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
#include "queue.h"
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

uint8_t  RxBuffer;          // 临时存放串口收到的这1个字�?
int16_t Vision_x = 0;      // 解析出来�?X 坐标 (给逻辑代码�?
int16_t Vision_y = 0;      // 解析出来�?Y 坐标 (给逻辑代码�?
uint8_t Vision_Status = 0; // 状态标记：1表示刚刚更新了数�?

// --- 状态机相关变量 ---
uint8_t rx_state = 0;      // 协议解析状�?
uint8_t rx_data_buf[4];    // 临时存放 X�? X�? Y�? Y�?
uint8_t rx_data_cnt = 0;   // 数据计数�?
typedef struct {
  uint8_t hw[4];
  float dist;
  float sr04;
  float pitch;
  float roll;
  float yaw;
} SensorSnapshot;

static SensorSnapshot g_sensor_snapshot;

static StaticQueue_t xCommandQueueBuffer;
static uint8_t ucCommandQueueStorage[10 * sizeof(uint8_t)];
static StaticQueue_t xVisionQueueBuffer;
static uint8_t ucVisionQueueStorage[5 * sizeof(VisionData_t)];
static StaticQueue_t xMotorTargetQueueBuffer;
static uint8_t ucMotorTargetQueueStorage[1 * sizeof(MotorTarget_t)];

static StaticTask_t xDefaultTaskTCB;
static StackType_t xDefaultTaskStack[128];
static StaticTask_t xControlTaskTCB;
static StackType_t xControlTaskStack[512];
static StaticTask_t xSensorTaskTCB;
static StackType_t xSensorTaskStack[128];
static StaticTask_t xVisionTaskTCB;
static StackType_t xVisionTaskStack[128];
static StaticTask_t xLogicTaskTCB;
static StackType_t xLogicTaskStack[128];
static StaticTask_t xStartUITaskTCB;
static StackType_t xStartUITaskStack[256];
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
QueueHandle_t MotorTargetQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void const * argument);
void StartControlTask(void const * argument);
void StartSensorTask(void const * argument);
void StartVisionTask(void const * argument);
void StartLogicTask(void const * argument);
void StartTask06(void const * argument);

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
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
  CommandQueueHandle = xQueueCreateStatic(10, sizeof(uint8_t),
                                          ucCommandQueueStorage, &xCommandQueueBuffer);
  VisionQueueHandle = xQueueCreateStatic(5, sizeof(VisionData_t),
                                         ucVisionQueueStorage, &xVisionQueueBuffer);
  MotorTargetQueueHandle = xQueueCreateStatic(1, sizeof(MotorTarget_t),
                                              ucMotorTargetQueueStorage, &xMotorTargetQueueBuffer);

  configASSERT(CommandQueueHandle != NULL);
  configASSERT(VisionQueueHandle != NULL);
  configASSERT(MotorTargetQueueHandle != NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128,
                    xDefaultTaskStack, &xDefaultTaskTCB);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ControlTask */
  osThreadStaticDef(ControlTask, StartControlTask, osPriorityHigh, 0, 512,
                    xControlTaskStack, &xControlTaskTCB);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of SensorTask */
  osThreadStaticDef(SensorTask, StartSensorTask, osPriorityAboveNormal, 0, 128,
                    xSensorTaskStack, &xSensorTaskTCB);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of VisionTask */
  osThreadStaticDef(VisionTask, StartVisionTask, osPriorityNormal, 0, 128,
                    xVisionTaskStack, &xVisionTaskTCB);
  VisionTaskHandle = osThreadCreate(osThread(VisionTask), NULL);

  /* definition and creation of LogicTask */
  osThreadStaticDef(LogicTask, StartLogicTask, osPriorityNormal, 0, 128,
                    xLogicTaskStack, &xLogicTaskTCB);
  LogicTaskHandle = osThreadCreate(osThread(LogicTask), NULL);

  /* definition and creation of StartUITask */
  osThreadStaticDef(StartUITask, StartTask06, osPriorityIdle, 0, 256,
                    xStartUITaskStack, &xStartUITaskTCB);
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

void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  TickType_t lastWakeTime = xTaskGetTickCount();
  MotorTarget_t target;

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // 开启编码器
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(MotorTargetQueueHandle, &target, 0) == pdTRUE)
    {
      pidMotor1Speed.target_val = target.left;
      pidMotor2Speed.target_val = target.right;
    }

    Encoder1Count=(short)__HAL_TIM_GET_COUNTER(&htim4);
    Encoder2Count=(short)__HAL_TIM_GET_COUNTER(&htim2);

    __HAL_TIM_SET_COUNTER(&htim4,0);
    __HAL_TIM_SET_COUNTER(&htim2,0);

    Motor1Speed = (float)Encoder1Count * 100 /9.6/11/4;
    Motor2Speed = -(float)Encoder2Count * 100 /9.6/11/4;

    mile+= 0.02*Motor1Speed*22;

    Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
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
  TickType_t lastWakeTime = xTaskGetTickCount();
  SensorSnapshot snap;

  /* Infinite loop */
  for(;;)
  {
    snap.hw[0] = READ_HW_OUT_1;
    snap.hw[1] = READ_HW_OUT_2;
    snap.hw[2] = READ_HW_OUT_3;
    snap.hw[3] = READ_HW_OUT_4;

    snap.dist = Get_Distance_Filtered();
    snap.sr04 = HC_SR04_Read();

    if (mpu_dmp_get_data(&snap.pitch, &snap.roll, &snap.yaw) != 0)
    {
      taskENTER_CRITICAL();
      snap.pitch = g_sensor_snapshot.pitch;
      snap.roll = g_sensor_snapshot.roll;
      snap.yaw = g_sensor_snapshot.yaw;
      taskEXIT_CRITICAL();
    }

    taskENTER_CRITICAL();
    g_sensor_snapshot = snap;
    g_read[0] = snap.hw[0];
    g_read[1] = snap.hw[1];
    g_read[2] = snap.hw[2];
    g_read[3] = snap.hw[3];
    dist = snap.dist;
    g_sr04_read = snap.sr04;
    pitch = snap.pitch;
    roll = snap.roll;
    yaw = snap.yaw;
    taskEXIT_CRITICAL();

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
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
  SensorSnapshot snap;
  /* Infinite loop */
  for(;;)
  {
    taskENTER_CRITICAL();
    snap = g_sensor_snapshot;
    taskEXIT_CRITICAL();

    if (xQueueReceive(CommandQueueHandle, &cmd_char, 0) == pdTRUE)
    {
      // 收到指令，开始干�?
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
        case 'H':
          mpu6050Movement.target_val += 90;
          break;
        case 'I':
          mpu6050Movement.target_val -= 90;
      }
    }
    switch (g_ucMode) {

      case 1:
        if (snap.hw[0] == 0 && snap.hw[1] == 0 && snap.hw[2] == 0 && snap.hw[3] == 0)
        {
          //motorPidSetSpeed(1,1);
          g_thisstate = 0;
        }

        else if (snap.hw[0] == 0  && snap.hw[1] == 0 && snap.hw[2] == 1 && snap.hw[3] == 0)
        {
          //motorPidSetSpeed(2,1);

          g_thisstate = 1;
        }
        else if (snap.hw[0] == 0  && snap.hw[1] == 1 && snap.hw[2] == 0 && snap.hw[3] == 0)
        {
          //motorPidSetSpeed(2,1);

          g_thisstate = -1;
        }
        else if (snap.hw[0] == 0  && snap.hw[1] == 0 && snap.hw[2] == 0 && snap.hw[3] == 1)
        {
          //motorPidSetSpeed(3,1);
          g_thisstate = 2;
        }

        else if (snap.hw[0] == 0  && snap.hw[1] == 0 && snap.hw[2] == 1 && snap.hw[3] == 1)
        {
          //motorPidSetSpeed(3,1);
          g_thisstate = 3;
        }
        else if (snap.hw[0] == 1 && snap.hw[1] == 0 &&  snap.hw[2] == 0 && snap.hw[3] == 0)
        {
          //motorPidSetSpeed(3,1);
          g_thisstate = -2;
        }

        else if (snap.hw[0] == 1  && snap.hw[1] == 1 && snap.hw[2] == 0 && snap.hw[3] == 0)
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
        if(snap.dist > 20) {
          motorPidSetSpeed(2, 2);
        } else {
          motorPidSetSpeed(0, 0);   osDelay(100);  // 停车
          motorPidSetSpeed(-1.5, -1.5); osDelay(300); // 后退
          motorPidSetSpeed(2, -2);  osDelay(400);  // 右转
          motorPidSetSpeed(0, 0);   osDelay(200);  // 停一下观�?
        }
        break;


      case 3:
        if(snap.sr04 < 60 )
        {
          g_follow_pid_out = PI_realize(&pidFollow, snap.sr04);


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
        g_fMPU6050YawMovePidOut = PID_realize(&mpu6050Movement, snap.yaw);

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
  SensorSnapshot snap;
  /* Infinite loop */
  for(;;)
  {
  // =========================================================

    // =========================================================
    // ADC读取放在这里或者SensorTask都可以，假�比较快直接读
    sprintf(oled_buf, "M:%d Bat:%.1fV", g_ucMode, adcGetBatteryVoltage());
    OLED_ShowString(0, 0, (uint8_t *)oled_buf, 12);

    taskENTER_CRITICAL();
    snap = g_sensor_snapshot;
    taskEXIT_CRITICAL();

    // =========================================================
    //  �?1 行�::显示 左右电机目标速度
    //  格式�?"L:100 R:100" (预留空格防止数字变短后有残留)
    // =========================================================
    // %.0f 表示不显示小数，节省空间�?-4.0f 表�ʾ左对齐占4�?
    sprintf(oled_buf, "L:%-4.0f R:%-4.0f", Motor1Speed, Motor2Speed);
    OLED_ShowString(0, 1, (uint8_t *)oled_buf, 12);


    // =========================================================
    //  �?2 行�::显示 超�?波距�?�?里��?    //  格式�?"D:120cm M:1.2"
    // =========================================================
    // g_dist �?SensorTask 更新�d��? (typo)
    sprintf(oled_buf, "D:%-3.0fcm M:%.1f", snap.dist, mile);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 12);


    // =========================================================
    //  �?3 行�::动态区�?(根据模式显示特有信息)
    // =========================================================
    if(g_ucMode == 5)
    {
        // 如果�?K210 模式，显示视觉坐�?
        sprintf(oled_buf, "K210 X: %d   ", Vision_x); //后面加空格是为了覆��掉旧�?
    }
    else if(g_ucMode == 4)
    {
        // 如果�?6050 走直线模式，�R��ʾ Yaw 角度
        sprintf(oled_buf, "Yaw: %-5.1f   ", snap.yaw);
    }
    else
    {
        // 其他模式（如循迹/遡蚜），可以显�ʾ状态或者是空的
        sprintf(oled_buf, "Running...    ");
    }
    OLED_ShowString(0, 3, (uint8_t *)oled_buf, 12);


    // =========================================================
    //  串口发�?(可选，不想刷屏可以把这里注�???, Actually maybe not
    // =========================================================
    // sprintf(oled_buf, "Mode:%d Dist:%.1f\r\n", g_ucMode, g_dist);
    // HAL_UART_Transmit(&huart3, (uint8_t *)oled_buf, strlen(oled_buf), 10);

    // �ˢ��频率�:ÿ��5��
    osDelay(200);
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for(;;)
  {
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  for(;;)
  {
  }
}

/* USER CODE END Application */






