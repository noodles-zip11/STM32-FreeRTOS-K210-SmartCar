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
#include "tim.h"
#include "gpio.h"
#include "adc.h"
#include "iwdg.h"
#include "queue.h"
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

volatile TickType_t g_last_cmd_tick = 0;
volatile TickType_t g_last_vision_tick = 0;
volatile uint8_t g_uart_manual_active = 0;



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
  uint32_t seq;
  uint8_t hw[4];
  float dist;
  float sr04;
  float pitch;
  float roll;
  float yaw;
} SensorSnapshot;

typedef enum {
  AVOID_IDLE = 0,
  AVOID_STOP,
  AVOID_BACK,
  AVOID_TURN,
  AVOID_PAUSE
}AvoidState;

AvoidState avoid_state = AVOID_IDLE;
TickType_t avoid_deadline = 0;

static inline void avoid_set_state(AvoidState state, uint32_t duration_ms)
{
  avoid_state = state;
  avoid_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
}



static volatile SensorSnapshot g_sensor_snapshot;

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
static StackType_t xStartUITaskStack[384];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef enum { MENU_TYPE_SUBMENU=0, MENU_TYPE_PARAM=1, MENU_TYPE_ACTION=2 } MenuItemType;
typedef enum { MENU_VALUE_NONE=0, MENU_VALUE_FLOAT=1, MENU_VALUE_INT8=2, MENU_VALUE_INT16=3, MENU_VALUE_INT32=4 } MenuValueType;
typedef struct {
  const char *name;
  uint8_t type;
  int8_t parent;
  uint8_t child_start;
  uint8_t child_count;
  uint8_t value_type;
  void *value_ptr;
  float step;
  float min;
  float max;
  uint8_t action_id;
} MenuItem;
typedef enum { UI_VIEW_MENU=0, UI_VIEW_STATUS=1, UI_VIEW_EDIT=2 } UiView;
typedef struct {
  uint8_t stable;
  uint8_t last_raw;
  uint32_t last_change;
  uint32_t press_tick;
  uint8_t long_sent;
} KeyState;

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

  /* Create the thread(s) */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128,
                    xDefaultTaskStack, &xDefaultTaskTCB);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadStaticDef(ControlTask, StartControlTask, osPriorityHigh, 0, 512,
                    xControlTaskStack, &xControlTaskTCB);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  osThreadStaticDef(SensorTask, StartSensorTask, osPriorityAboveNormal, 0, 128,
                    xSensorTaskStack, &xSensorTaskTCB);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  osThreadStaticDef(VisionTask, StartVisionTask, osPriorityNormal, 0, 128,
                    xVisionTaskStack, &xVisionTaskTCB);
  VisionTaskHandle = osThreadCreate(osThread(VisionTask), NULL);

  osThreadStaticDef(LogicTask, StartLogicTask, osPriorityNormal, 0, 128,
                    xLogicTaskStack, &xLogicTaskTCB);
  LogicTaskHandle = osThreadCreate(osThread(LogicTask), NULL);

  osThreadStaticDef(StartUITask, StartTask06, osPriorityLow, 0, 384,
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
  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t lastTick = lastWakeTime;
  MotorTarget_t target;
  float dt = 0.01f;

  static float m1_speed_f = 0.0f;
  static float m2_speed_f = 0.0f;
  //更精准的�?dt
  // TickType_t now = xTaskGetTickCount();
  // float dt = (now - lastTick) * 0.001f;
  // lastTick = now;

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

    TickType_t now = xTaskGetTickCount();
    dt = (now - lastTick) * 0.001f;
    if (dt <= 0.0f) dt = 0.01f;
    lastTick = now;

    Encoder1Count=(short)__HAL_TIM_GET_COUNTER(&htim4);
    Encoder2Count=(short)__HAL_TIM_GET_COUNTER(&htim2);

    __HAL_TIM_SET_COUNTER(&htim4,0);
    __HAL_TIM_SET_COUNTER(&htim2,0);

    // Motor1Speed = (float)Encoder1Count * 100 /9.6/11/4;
    // Motor2Speed = -(float)Encoder2Count * 100 /9.6/11/4;


    float rev1 = (float)Encoder1Count / (ENC_PPR * GEAR_RATIO);
    float rev2 = (float)Encoder2Count / (ENC_PPR * GEAR_RATIO);

    Motor1Speed = rev1 / dt;
    Motor2Speed = -rev2 / dt;

    // 一阶低�?
    m1_speed_f += SPEED_LPF_A * (Motor1Speed - m1_speed_f);
    m2_speed_f += SPEED_LPF_A * (Motor2Speed - m2_speed_f);

    // PID 用滤波后�?
    Motor_Set(
      PID_realize(&pidMotor1Speed, m1_speed_f, dt),
      PID_realize(&pidMotor2Speed, m2_speed_f, dt)
    );
    HAL_IWDG_Refresh(&hiwdg);


    // mile+= 0.02*Motor1Speed*22;
    //
    //
    // Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed,dt),PID_realize(&pidMotor2Speed,Motor2Speed,dt));

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
    g_sensor_snapshot.seq++;
    g_sensor_snapshot = snap;
    g_sensor_snapshot.seq++;
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

  VisionData_t data;
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(VisionQueueHandle, &data, pdMS_TO_TICKS(20)) == pdTRUE)
    {
      // 这里处理 data.x / data.y
      // 例如更新目标或者保存到全局结构�?
      // last_vision_tick = xTaskGetTickCount();
    }
    //osDelay(10);
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
  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t lastTick = lastWakeTime;
  float dt = 0.05f;


  g_last_cmd_tick = xTaskGetTickCount();
  g_last_vision_tick = g_last_cmd_tick;

  /* Infinite loop */
  for(;;)
  {
    
    TickType_t now = xTaskGetTickCount();
    dt = (now - lastTick) * 0.001f;
    if (dt <= 0.0f) dt = 0.05f;
    lastTick = now;

    SensorSnapshot snap1, snap2;
    do {
      snap1 = g_sensor_snapshot;
      snap2 = g_sensor_snapshot; // 2. 尝试读第二次

    } while ((snap1.seq != snap2.seq) || (snap1.seq & 1));
    snap = snap1;


    if (xQueueReceive(CommandQueueHandle, &cmd_char, 0) == pdTRUE)
    {
      // 收到指令，开始干�?
      g_uart_manual_active = 1;
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
          g_uart_manual_active = 0;
          break;
        case 'K':
          g_ucMode = 0;
          g_uart_manual_active = 0;
          break;
        case 'H':
          mpu6050Movement.target_val += 90;
          break;
        case 'I':
          mpu6050Movement.target_val -= 90;
      }
    }
    if (g_uart_manual_active && (now - g_last_cmd_tick) > pdMS_TO_TICKS(1000)) {
      motorPidSetSpeed(0, 0);
      g_uart_manual_active = 0;
    }
    if ((now - g_last_vision_tick) > pdMS_TO_TICKS(500)) {
      Vision_Status = 0;
    }

    switch (g_ucMode) {
      case 0:
        if (!g_uart_manual_active) {
          motorPidSetSpeed(0,0);
        }
        break;

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

        g_pid_out = PID_realize(&pid_pidHW_Tracking,g_thisstate,dt);

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

        TickType_t  now = xTaskGetTickCount();

        if(snap.dist > 20)
        {
          avoid_state = AVOID_IDLE;
          motorPidSetSpeed(2, 2);
          break;
        }
        switch (avoid_state)
        {
          case AVOID_IDLE:
            motorPidSetSpeed(0, 0);
            avoid_set_state(AVOID_BACK, 100); // 停车 100ms
            break;

          case AVOID_BACK:
            if (now >= avoid_deadline) {
              motorPidSetSpeed(-1.5f, -1.5f);
              avoid_set_state(AVOID_TURN, 300); // 后退 300ms
            }
            break;

          case AVOID_TURN:
            if (now >= avoid_deadline) {
              motorPidSetSpeed(2, -2);
              avoid_set_state(AVOID_PAUSE, 400); // 右转 400ms
            }
            break;

          case AVOID_PAUSE:
            if (now >= avoid_deadline) {
              motorPidSetSpeed(0, 0);
              avoid_set_state(AVOID_IDLE, 200); // 停一�?
            }
            break;
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
        g_fMPU6050YawMovePidOut = PID_realize(&mpu6050Movement, snap.yaw,dt);

        g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;

        g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;

        if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;
        if(g_fMPU6050YawMovePidOut1 < 0 ) g_fMPU6050YawMovePidOut1 =0;
        if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
        if(g_fMPU6050YawMovePidOut2 < 0) g_fMPU6050YawMovePidOut2 = 0;

        motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
        break;
        
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));
  }
  /* USER CODE END StartLogicTask */
}

static MenuItem g_menu_items[] = {
  {"Root", MENU_TYPE_SUBMENU, -1, 1, 5, MENU_VALUE_NONE, 0, 0, 0, 0, 0},
  {"Status", MENU_TYPE_ACTION, 0, 0, 0, MENU_VALUE_NONE, 0, 0, 0, 0, 1},
  {"Mode", MENU_TYPE_PARAM, 0, 0, 0, MENU_VALUE_INT8, &g_ucMode, 1.0f, 0.0f, 5.0f, 0},
  {"PID M1", MENU_TYPE_SUBMENU, 0, 6, 3, MENU_VALUE_NONE, 0, 0, 0, 0, 0},
  {"PID M2", MENU_TYPE_SUBMENU, 0, 9, 3, MENU_VALUE_NONE, 0, 0, 0, 0, 0},
  {"PID Track", MENU_TYPE_SUBMENU, 0, 12, 3, MENU_VALUE_NONE, 0, 0, 0, 0, 0},
  {"M1 Kp", MENU_TYPE_PARAM, 3, 0, 0, MENU_VALUE_FLOAT, &pidMotor1Speed.kp, 0.1f, -50.0f, 50.0f, 0},
  {"M1 Ki", MENU_TYPE_PARAM, 3, 0, 0, MENU_VALUE_FLOAT, &pidMotor1Speed.ki, 0.01f, -10.0f, 10.0f, 0},
  {"M1 Kd", MENU_TYPE_PARAM, 3, 0, 0, MENU_VALUE_FLOAT, &pidMotor1Speed.kd, 0.1f, -50.0f, 50.0f, 0},
  {"M2 Kp", MENU_TYPE_PARAM, 4, 0, 0, MENU_VALUE_FLOAT, &pidMotor2Speed.kp, 0.1f, -50.0f, 50.0f, 0},
  {"M2 Ki", MENU_TYPE_PARAM, 4, 0, 0, MENU_VALUE_FLOAT, &pidMotor2Speed.ki, 0.01f, -10.0f, 10.0f, 0},
  {"M2 Kd", MENU_TYPE_PARAM, 4, 0, 0, MENU_VALUE_FLOAT, &pidMotor2Speed.kd, 0.1f, -50.0f, 50.0f, 0},
  {"TR Kp", MENU_TYPE_PARAM, 5, 0, 0, MENU_VALUE_FLOAT, &pid_pidHW_Tracking.kp, 0.1f, -50.0f, 50.0f, 0},
  {"TR Ki", MENU_TYPE_PARAM, 5, 0, 0, MENU_VALUE_FLOAT, &pid_pidHW_Tracking.ki, 0.01f, -10.0f, 10.0f, 0},
  {"TR Kd", MENU_TYPE_PARAM, 5, 0, 0, MENU_VALUE_FLOAT, &pid_pidHW_Tracking.kd, 0.1f, -50.0f, 50.0f, 0}
};

static int32_t ui_get_int(const MenuItem *item)
{
  if (item->value_type == MENU_VALUE_INT8) return *(int8_t *)item->value_ptr;
  if (item->value_type == MENU_VALUE_INT16) return *(int16_t *)item->value_ptr;
  if (item->value_type == MENU_VALUE_INT32) return *(int32_t *)item->value_ptr;
  return 0;
}

static void ui_set_int(const MenuItem *item, int32_t v)
{
  if (item->value_type == MENU_VALUE_INT8) *(int8_t *)item->value_ptr = (int8_t)v;
  else if (item->value_type == MENU_VALUE_INT16) *(int16_t *)item->value_ptr = (int16_t)v;
  else if (item->value_type == MENU_VALUE_INT32) *(int32_t *)item->value_ptr = (int32_t)v;
}

static void ui_key_update(KeyState *k, uint8_t raw, uint32_t now, uint32_t debounce_ms, uint32_t long_ms, uint8_t *short_evt, uint8_t *long_evt)
{
  if (raw != k->last_raw)
  {
    k->last_raw = raw;
    k->last_change = now;
  }
  else if ((now - k->last_change) >= debounce_ms && raw != k->stable)
  {
    k->stable = raw;
    if (k->stable)
    {
      k->press_tick = now;
      k->long_sent = 0;
    }
    else
    {
      if (!k->long_sent) *short_evt = 1;
    }
  }

  if (k->stable && !k->long_sent && (now - k->press_tick) >= long_ms)
  {
    *long_evt = 1;
    k->long_sent = 1;
  }
}

static void ui_format_value(const MenuItem *item, char *buf, uint8_t len)
{
  if (item->value_type == MENU_VALUE_FLOAT)
  {
    snprintf(buf, len, "%.2f", *(float *)item->value_ptr);
  }
  else if (item->value_type != MENU_VALUE_NONE)
  {
    snprintf(buf, len, "%ld", (long)ui_get_int(item));
  }
  else
  {
    buf[0] = '\0';
  }
}

static uint8_t ui_find_child_cursor(uint8_t parent, uint8_t child_index)
{
  uint8_t start = g_menu_items[parent].child_start;
  uint8_t count = g_menu_items[parent].child_count;
  uint8_t i;
  for (i = 0; i < count; i++)
  {
    if (start + i == child_index) return i;
  }
  return 0;
}

static void ui_draw_menu(uint8_t parent, uint8_t cursor, uint8_t view_offset, uint8_t edit_mode, uint8_t edit_index)
{
  uint8_t start = g_menu_items[parent].child_start;
  uint8_t count = g_menu_items[parent].child_count;
  uint8_t line;
  OLED_Clear();
  for (line = 0; line < 4; line++)
  {
    uint8_t idx = start + view_offset + line;
    if (idx >= start + count) break;
    MenuItem *item = &g_menu_items[idx];
    char linebuf[24];
    char valbuf[12];
    char mark = ' ';
    if (idx == start + cursor) mark = '>';
    if (edit_mode && idx == edit_index) mark = '*';
    if (item->type == MENU_TYPE_PARAM)
    {
      ui_format_value(item, valbuf, sizeof(valbuf));
      snprintf(linebuf, sizeof(linebuf), "%c%s:%s", mark, item->name, valbuf);
    }
    else
    {
      snprintf(linebuf, sizeof(linebuf), "%c%s", mark, item->name);
    }
    OLED_ShowString(0, line, (uint8_t *)linebuf, 12);
  }
}

static void ui_draw_status(void)
{
  char line[32];
  OLED_Clear();
  snprintf(line, sizeof(line), "Mode:%d", g_ucMode);
  OLED_ShowString(0, 0, (uint8_t *)line, 12);
  snprintf(line, sizeof(line), "L:%.1f R:%.1f", Motor1Speed, Motor2Speed);
  OLED_ShowString(0, 1, (uint8_t *)line, 12);
  snprintf(line, sizeof(line), "D:%.1f M:%.1f", g_sensor_snapshot.dist, mile);
  OLED_ShowString(0, 2, (uint8_t *)line, 12);
  snprintf(line, sizeof(line), "Yaw:%.1f", g_sensor_snapshot.yaw);
  OLED_ShowString(0, 3, (uint8_t *)line, 12);
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

  KeyState key1 = {0};
  KeyState key2 = {0};
  UiView view = UI_VIEW_MENU;
  uint8_t current_parent = 0;
  uint8_t cursor = 0;
  uint8_t view_offset = 0;
  uint8_t edit_index = 0;
  float edit_backup_f = 0.0f;
  int32_t edit_backup_i = 0;
  uint32_t last_draw = 0;

  for(;;)
  {
    uint32_t now = HAL_GetTick();
    uint8_t key1_short = 0;
    uint8_t key1_long = 0;
    uint8_t key2_short = 0;
    uint8_t key2_long = 0;
    uint8_t need_redraw = 0;

    uint8_t raw1 = (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET);
    uint8_t raw2 = (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET);

    ui_key_update(&key1, raw1, now, 15, 400, &key1_short, &key1_long);
    ui_key_update(&key2, raw2, now, 15, 400, &key2_short, &key2_long);

    if (view == UI_VIEW_MENU)
    {
      uint8_t count = g_menu_items[current_parent].child_count;
      uint8_t start = g_menu_items[current_parent].child_start;
      if (key1_short)
      {
        if (cursor == 0) cursor = count - 1; else cursor--;
        need_redraw = 1;
      }
      if (key2_short)
      {
        cursor++;
        if (cursor >= count) cursor = 0;
        need_redraw = 1;
      }
      if (cursor < view_offset) view_offset = cursor;
      if (cursor >= view_offset + 4) view_offset = cursor - 3;

      if (key1_long)
      {
        uint8_t idx = start + cursor;
        MenuItem *item = &g_menu_items[idx];
        if (item->type == MENU_TYPE_SUBMENU)
        {
          current_parent = idx;
          cursor = 0;
          view_offset = 0;
          need_redraw = 1;
        }
        else if (item->type == MENU_TYPE_PARAM)
        {
          view = UI_VIEW_EDIT;
          edit_index = idx;
          if (item->value_type == MENU_VALUE_FLOAT) edit_backup_f = *(float *)item->value_ptr;
          else edit_backup_i = ui_get_int(item);
          need_redraw = 1;
        }
        else if (item->type == MENU_TYPE_ACTION)
        {
          if (item->action_id == 1) view = UI_VIEW_STATUS;
          need_redraw = 1;
        }
      }

      if (key2_long)
      {
        if (current_parent != 0)
        {
          uint8_t parent = g_menu_items[current_parent].parent;
          cursor = ui_find_child_cursor(parent, current_parent);
          current_parent = parent;
          view_offset = 0;
          if (cursor >= 4) view_offset = cursor - 3;
          need_redraw = 1;
        }
      }

      if (need_redraw || (now - last_draw) > 200)
      {
        ui_draw_menu(current_parent, cursor, view_offset, 0, edit_index);
        last_draw = now;
      }
    }
    else if (view == UI_VIEW_EDIT)
    {
      MenuItem *item = &g_menu_items[edit_index];
      if (key1_short || key2_short)
      {
        if (item->value_type == MENU_VALUE_FLOAT)
        {
          float v = *(float *)item->value_ptr;
          float step = item->step;
          if (key1_short) v += step;
          if (key2_short) v -= step;
          if (v > item->max) v = item->max;
          if (v < item->min) v = item->min;
          *(float *)item->value_ptr = v;
        }
        else
        {
          int32_t v = ui_get_int(item);
          int32_t step = (int32_t)item->step;
          if (step <= 0) step = 1;
          if (key1_short) v += step;
          if (key2_short) v -= step;
          if (v > (int32_t)item->max) v = (int32_t)item->max;
          if (v < (int32_t)item->min) v = (int32_t)item->min;
          ui_set_int(item, v);
        }
        need_redraw = 1;
      }
      if (key1_long)
      {
        view = UI_VIEW_MENU;
        need_redraw = 1;
      }
      if (key2_long)
      {
        if (item->value_type == MENU_VALUE_FLOAT) *(float *)item->value_ptr = edit_backup_f;
        else ui_set_int(item, edit_backup_i);
        view = UI_VIEW_MENU;
        need_redraw = 1;
      }

      if (need_redraw || (now - last_draw) > 200)
      {
        ui_draw_menu(current_parent, cursor, view_offset, 1, edit_index);
        last_draw = now;
      }
    }
    else if (view == UI_VIEW_STATUS)
    {
      if (key2_long || key1_long)
      {
        view = UI_VIEW_MENU;
        need_redraw = 1;
      }
      if (need_redraw || (now - last_draw) > 200)
      {
        ui_draw_status();
        last_draw = now;
      }
    }

    osDelay(20);
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void vApplicationMallocFailedHook(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  taskDISABLE_INTERRUPTS();
  for(;;)
  {
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  taskDISABLE_INTERRUPTS();
  for(;;)
  {
  }
}

/* USER CODE END Application */

