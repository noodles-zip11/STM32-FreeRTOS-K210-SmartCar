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
#include "math.h"
//#include "HC_SR04.h"
// #include "mpu6050.h"
 #include "inv_mpu.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
// #include "adc.h"
#include "iwdg.h"
#include "queue.h"
#include "settings.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//pid变量
float g_line_base_speed = 2.0f;
float g_line_search_speed = 1.5f;
float g_line_max_speed = 4.0f;
float g_line_min_speed = 0.5f;

//电机测速和设置速度
short Encoder1Count =0 ;
short Encoder2Count =0 ;
float Motor1Speed = 0.00;
float Motor2Speed = 0.00;
extern tpid pidMotor1Speed;
extern tpid pidMotor2Speed;

//里程
float mile =0.00 ;

//串口手动控制超时
volatile TickType_t g_last_cmd_tick = 0;
volatile uint8_t g_uart_manual_active = 0;

//跟踪功能
float g_follow_pid_out=0.0;
float dist;


extern tpid pidFollow;
extern tpid mpu6050Movement;

//循迹功能变量
extern tpid pid_pidHW_Tracking;
uint8_t g_read[35];
int8_t g_thisstate = 0;
int8_t g_laststate = 0;
float g_pid_out = 0.0;
float g_pid_out1 = 0.0;
float g_pid_out2 = 0.0;

#define LINE_VISION_FRESH_MS      150U
#define LINE_VISION_STOP_MS       800U
#define LINE_VISION_QUALITY_TH    350
#define LINE_VISION_ERR_LPF_A     0.30f
#define LINE_VISION_SLOW_GAIN     0.80f
#define LINE_VISION_SEARCH_TURN   0.60f
#define LINE_VISION_DEBUG_UART1_EN 1
#define LINE_VISION_DEBUG_MS      100U

//蓝牙接收
uint8_t g_ucusrtrecivedate;

//模式切换
volatile uint8_t g_ucMode = 0 ;

//走直线
float  g_fMPU6050YawMovePidOut = 0.00f;
float  g_fMPU6050YawMovePidOut1 = 0.00f;
float  g_fMPU6050YawMovePidOut2 = 0.00f;

//角
float pitch,roll,yaw;

//单字节接收缓冲区（视觉）
uint8_t  RxBuffer;
//vofa
static TickType_t g_vofa_tx_tick = 0;

//数据快照结构体
typedef struct {
  uint32_t seq;
  uint8_t hw[4];
  float dist;
  float sr04;
  float pitch;
  float roll;
  float yaw;
} SensorSnapshot;

//跟随结构体
typedef enum {
  AVOID_IDLE = 0,
  AVOID_STOP,
  AVOID_BACK,
  AVOID_TURN,
  AVOID_PAUSE
}AvoidState;

//跟随
AvoidState avoid_state = AVOID_IDLE;
TickType_t avoid_deadline = 0;

//跟随时传递状态和停留时间函数
static inline void avoid_set_state(AvoidState state, uint32_t duration_ms)
{
  avoid_state = state;
  avoid_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
}

//全局数据快照
static volatile SensorSnapshot g_sensor_snapshot;

typedef enum {
  LINE_VISION_TRACK = 0,
  LINE_VISION_SEARCH,
  LINE_VISION_LOST_STOP
} LineVisionState;

typedef struct {
  float err_f;
  int16_t quality;
  TickType_t tick;
  int8_t sign;
} VisionRuntime;

static volatile VisionRuntime g_vision_runtime = {0.0f, 0, 0, 1};
static LineVisionState g_line_vision_state = LINE_VISION_LOST_STOP;

//准备队列静态内存和创建静态队列
static StaticQueue_t xCommandQueueBuffer;
static uint8_t ucCommandQueueStorage[10 * sizeof(uint8_t)];
static StaticQueue_t xVisionQueueBuffer;
static uint8_t ucVisionQueueStorage[1 * sizeof(VisionData_t)];
static StaticQueue_t xMotorTargetQueueBuffer;
static uint8_t ucMotorTargetQueueStorage[1 * sizeof(MotorTarget_t)];

//准备任务静态内存和TCB
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
//ui菜单
#define UI_LIST_LINES 4
#define UI_LINE_CHARS 21
#define UI_KEY_DEBOUNCE_MS 5
#define UI_KEY_LONG_MS 350
#define UI_MENU_REFRESH_MS 200
#define UI_EDIT_REFRESH_MS 80
#define UI_STATUS_REFRESH_MS 120

#define MENU_ACTION_STATUS 1

typedef enum { MENU_TYPE_SUBMENU=0, MENU_TYPE_PARAM=1, MENU_TYPE_ACTION=2 } MenuItemType;
typedef enum { MENU_VALUE_NONE=0, MENU_VALUE_FLOAT=1, MENU_VALUE_INT8=2, MENU_VALUE_INT16=3, MENU_VALUE_INT32=4 } MenuValueType;
typedef enum { UI_VIEW_MENU=0, UI_VIEW_STATUS=1, UI_VIEW_EDIT=2 } UiView;

typedef struct {
  const char *name;
  MenuItemType type;
  int8_t parent;
  MenuValueType value_type;
  void *value_ptr;
  float step;
  float min;
  float max;
  uint8_t action_id;
} MenuItem;

typedef struct {
  uint8_t stable;
  uint8_t last_raw;
  uint32_t last_change;
  uint32_t press_tick;
  uint8_t long_sent;
} KeyState;

typedef struct {
  uint8_t short_press;
  uint8_t long_press;
} KeyEvent;

enum {
  MENU_IDX_ROOT = 0,
  MENU_IDX_STATUS,
  MENU_IDX_MODE,
  MENU_IDX_PID_M1,
  MENU_IDX_PID_M2,
  MENU_IDX_PID_TRACK,
  MENU_IDX_LINE,
  MENU_IDX_M1_KP,
  MENU_IDX_M1_KI,
  MENU_IDX_M1_KD,
  MENU_IDX_M2_KP,
  MENU_IDX_M2_KI,
  MENU_IDX_M2_KD,
  MENU_IDX_TR_KP,
  MENU_IDX_TR_KI,
  MENU_IDX_TR_KD,
  MENU_IDX_LINE_BASE,
  MENU_IDX_LINE_SEARCH,
  MENU_IDX_LINE_MAX,
  MENU_IDX_LINE_MIN,
  MENU_IDX_COUNT
};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static const MenuItem g_menu_items[MENU_IDX_COUNT] = {
  {"Root", MENU_TYPE_SUBMENU, -1, MENU_VALUE_NONE, 0, 0.0f, 0.0f, 0.0f, 0},
  {"Status", MENU_TYPE_ACTION, MENU_IDX_ROOT, MENU_VALUE_NONE, 0, 0.0f, 0.0f, 0.0f, MENU_ACTION_STATUS},
  {"Mode", MENU_TYPE_PARAM, MENU_IDX_ROOT, MENU_VALUE_INT8, &g_ucMode, 1.0f, 0.0f, 4.0f, 0},
  {"PID M1", MENU_TYPE_SUBMENU, MENU_IDX_ROOT, MENU_VALUE_NONE, 0, 0.0f, 0.0f, 0.0f, 0},
  {"PID M2", MENU_TYPE_SUBMENU, MENU_IDX_ROOT, MENU_VALUE_NONE, 0, 0.0f, 0.0f, 0.0f, 0},
  {"PID Track", MENU_TYPE_SUBMENU, MENU_IDX_ROOT, MENU_VALUE_NONE, 0, 0.0f, 0.0f, 0.0f, 0},
  {"Line", MENU_TYPE_SUBMENU, MENU_IDX_ROOT, MENU_VALUE_NONE, 0, 0.0f, 0.0f, 0.0f, 0},
  {"M1 Kp", MENU_TYPE_PARAM, MENU_IDX_PID_M1, MENU_VALUE_FLOAT, &pidMotor1Speed.kp, 0.1f, -50.0f, 50.0f, 0},
  {"M1 Ki", MENU_TYPE_PARAM, MENU_IDX_PID_M1, MENU_VALUE_FLOAT, &pidMotor1Speed.ki, 0.01f, -10.0f, 10.0f, 0},
  {"M1 Kd", MENU_TYPE_PARAM, MENU_IDX_PID_M1, MENU_VALUE_FLOAT, &pidMotor1Speed.kd, 0.1f, -50.0f, 50.0f, 0},
  {"M2 Kp", MENU_TYPE_PARAM, MENU_IDX_PID_M2, MENU_VALUE_FLOAT, &pidMotor2Speed.kp, 0.1f, -50.0f, 50.0f, 0},
  {"M2 Ki", MENU_TYPE_PARAM, MENU_IDX_PID_M2, MENU_VALUE_FLOAT, &pidMotor2Speed.ki, 0.01f, -10.0f, 10.0f, 0},
  {"M2 Kd", MENU_TYPE_PARAM, MENU_IDX_PID_M2, MENU_VALUE_FLOAT, &pidMotor2Speed.kd, 0.1f, -50.0f, 50.0f, 0},
  {"TR Kp", MENU_TYPE_PARAM, MENU_IDX_PID_TRACK, MENU_VALUE_FLOAT, &pid_pidHW_Tracking.kp, 0.1f, -50.0f, 50.0f, 0},
  {"TR Ki", MENU_TYPE_PARAM, MENU_IDX_PID_TRACK, MENU_VALUE_FLOAT, &pid_pidHW_Tracking.ki, 0.01f, -10.0f, 10.0f, 0},
  {"TR Kd", MENU_TYPE_PARAM, MENU_IDX_PID_TRACK, MENU_VALUE_FLOAT, &pid_pidHW_Tracking.kd, 0.1f, -50.0f, 50.0f, 0},
  {"Base Spd", MENU_TYPE_PARAM, MENU_IDX_LINE, MENU_VALUE_FLOAT, &g_line_base_speed, 0.1f, 0.0f, 8.0f, 0},
  {"Search Spd", MENU_TYPE_PARAM, MENU_IDX_LINE, MENU_VALUE_FLOAT, &g_line_search_speed, 0.1f, 0.0f, 8.0f, 0},
  {"Max Spd", MENU_TYPE_PARAM, MENU_IDX_LINE, MENU_VALUE_FLOAT, &g_line_max_speed, 0.1f, 0.0f, 8.0f, 0},
  {"Min Spd", MENU_TYPE_PARAM, MENU_IDX_LINE, MENU_VALUE_FLOAT, &g_line_min_speed, 0.1f, 0.0f, 8.0f, 0}
};

static const uint8_t g_menu_count = (uint8_t)(sizeof(g_menu_items) / sizeof(g_menu_items[0]));
//ui菜单

/* USER CODE END Variables */
//创建句柄
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
//声明函数
void StartDefaultTask(void const * argument);
void StartControlTask(void const * argument);
void StartSensorTask(void const * argument);
void StartVisionTask(void const * argument);
void StartLogicTask(void const * argument);
void StartTask06(void const * argument);

static int32_t ui_get_int(const MenuItem *item);
static void ui_set_int(const MenuItem *item, int32_t v);
static void ui_key_update(KeyState *k, uint8_t raw, uint32_t now, KeyEvent *evt);
static void ui_format_value(const MenuItem *item, char *buf, uint8_t len);
static uint8_t ui_menu_child_count(int8_t parent);
static int16_t ui_menu_child_index(int8_t parent, uint8_t child_pos);
static uint8_t ui_menu_child_pos(int8_t parent, uint8_t child_index);
static void ui_draw_menu(int8_t parent, uint8_t cursor, uint8_t view_offset, uint8_t edit_mode, uint8_t edit_index);
static void ui_draw_line(uint8_t row, const char *text);
static void ui_draw_status(void);
//vofa
static void vofa_send_waveform(float target_speed, float actual_speed);
static float clampf(float v, float vmin, float vmax);

/* USER CODE END FunctionPrototypes */
// void StartDefaultTask(void const * argument);
// void StartControlTask(void const * argument);
// void StartSensorTask(void const * argument);
// void StartVisionTask(void const * argument);
// void StartLogicTask(void const * argument);
// void StartTask06(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
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

  //创造队列
  CommandQueueHandle = xQueueCreateStatic(10, sizeof(uint8_t),
                                          ucCommandQueueStorage, &xCommandQueueBuffer);
  VisionQueueHandle = xQueueCreateStatic(1, sizeof(VisionData_t),
                                         ucVisionQueueStorage, &xVisionQueueBuffer);
  MotorTargetQueueHandle = xQueueCreateStatic(1, sizeof(MotorTarget_t),
                                              ucMotorTargetQueueStorage, &xMotorTargetQueueBuffer);
  configASSERT(CommandQueueHandle != NULL);
  configASSERT(VisionQueueHandle != NULL);
  configASSERT(MotorTargetQueueHandle != NULL);

  /* Create the thread(s) */
//优先级排行
  //控制 1
  //传感器传数据 2
  //视觉 逻辑控制 3
  //ui 4
  //任务队列和传入句柄
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128,
                    xDefaultTaskStack, &xDefaultTaskTCB);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadStaticDef(ControlTask, StartControlTask, osPriorityHigh, 0, 512,
                    xControlTaskStack, &xControlTaskTCB);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  osThreadStaticDef(SensorTask, StartSensorTask, osPriorityNormal, 0, 128,
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
  osThreadSetPriority(StartUITaskHandle, osPriorityNormal);
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
    settings_service();
    osDelay(20);
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
  TickType_t lastWakeTime = xTaskGetTickCount();// vTaskDelayUntil 使用的参考时间
  TickType_t lastTick = lastWakeTime; // 上一次计算 dt 的时间戳
  MotorTarget_t target;  // 队列里接收到的目标速度
  float dt;  // 控制周期（秒）

  static float m1_speed_f = 0.0f; // 电机1速度低通滤波后的值
  static float m2_speed_f = 0.0f; // 电机2速度低通滤波后的值
  static float last_target_m1 = 0.0f;
  static float last_target_m2 = 0.0f;



  // 开启编码器
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  /* Infinite loop */
  for(;;)
  {
    //有队列有值就传递target，反向清数据
    if (xQueueReceive(MotorTargetQueueHandle, &target, 0) == pdTRUE)
    {
      float new_target_m1 = target.left;
      float new_target_m2 = target.right;

      if ((last_target_m1 > 0.0f && new_target_m1 < 0.0f) ||
          (last_target_m1 < 0.0f && new_target_m1 > 0.0f))
      {
        PID_Reset(&pidMotor1Speed);
      }

      if ((last_target_m2 > 0.0f && new_target_m2 < 0.0f) ||
          (last_target_m2 < 0.0f && new_target_m2 > 0.0f))
      {
        PID_Reset(&pidMotor2Speed);
      }

      pidMotor1Speed.target_val = new_target_m1;
      pidMotor2Speed.target_val = new_target_m2;
      last_target_m1 = new_target_m1;
      last_target_m2 = new_target_m2;
    }

    //精准计算控制周期
    TickType_t now = xTaskGetTickCount();
    dt = (now - lastTick) * 0.001f;
    if (dt <= 0.0f) dt = 0.01f;
    lastTick = now;

    //获得圈数
    Encoder1Count=(short)__HAL_TIM_GET_COUNTER(&htim4);
    Encoder2Count=(short)__HAL_TIM_GET_COUNTER(&htim2);

    __HAL_TIM_SET_COUNTER(&htim4,0);
    __HAL_TIM_SET_COUNTER(&htim2,0);

    //计算当前速度
    float rev1 = (float)Encoder1Count / (ENC_PPR * GEAR_RATIO);
    float rev2 = (float)Encoder2Count / (ENC_PPR * GEAR_RATIO);

    Motor1Speed = rev1 / dt;
    Motor2Speed = -rev2 / dt;

    m1_speed_f += SPEED_LPF_A * (Motor1Speed - m1_speed_f);
    m2_speed_f += SPEED_LPF_A * (Motor2Speed - m2_speed_f);

   //设置速度
    Motor_Set(
      PID_realize(&pidMotor1Speed, m1_speed_f, dt),
      PID_realize(&pidMotor2Speed, m2_speed_f, dt)
    );

    //vofa
    // if ((now - g_vofa_tx_tick) >= pdMS_TO_TICKS(50))
    // {
    //   float target_speed = 0.5f * (pidMotor1Speed.target_val + pidMotor2Speed.target_val);
    //   float actual_speed = 0.5f * (m1_speed_f + m2_speed_f);
    //   vofa_send_waveform(target_speed, actual_speed);
    //   g_vofa_tx_tick = now;
    // }

    HAL_IWDG_Refresh(&hiwdg);  // 周期性刷新 IWDG，避免任务阻塞导致系统复位

    //绝对延时
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
    //读循迹电平
    snap.hw[0] = READ_HW_OUT_1;
    snap.hw[1] = READ_HW_OUT_2;
    snap.hw[2] = READ_HW_OUT_3;
    snap.hw[3] = READ_HW_OUT_4;

    //读距离（统一一条超声数据链，避免同周期重复触发）
    snap.sr04 = Get_Distance_Filtered();
    snap.dist = snap.sr04;

    //读取失败，就回退到上一次的有效值
    if (mpu_dmp_get_data(&snap.pitch, &snap.roll, &snap.yaw) != 0)
    {
      taskENTER_CRITICAL();
      snap.pitch = g_sensor_snapshot.pitch;
      snap.roll = g_sensor_snapshot.roll;
      snap.yaw = g_sensor_snapshot.yaw;
      taskEXIT_CRITICAL();
    }

    //关中断打包数据
    taskENTER_CRITICAL();
    g_sensor_snapshot.seq++;
    if ((g_sensor_snapshot.seq & 1U) == 0U) g_sensor_snapshot.seq++;
    g_sensor_snapshot.hw[0] = snap.hw[0];
    g_sensor_snapshot.hw[1] = snap.hw[1];
    g_sensor_snapshot.hw[2] = snap.hw[2];
    g_sensor_snapshot.hw[3] = snap.hw[3];
    g_sensor_snapshot.dist = snap.dist;
    g_sensor_snapshot.sr04 = snap.sr04;
    g_sensor_snapshot.pitch = snap.pitch;
    g_sensor_snapshot.roll = snap.roll;
    g_sensor_snapshot.yaw = snap.yaw;
    g_sensor_snapshot.seq++;
    g_read[0] = snap.hw[0];
    g_read[1] = snap.hw[1];
    g_read[2] = snap.hw[2];
    g_read[3] = snap.hw[3];
    dist = snap.dist;
    pitch = snap.pitch;
    roll = snap.roll;
    yaw = snap.yaw;
    taskEXIT_CRITICAL();

    //绝对延时
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
  VisionData_t latest;
  TickType_t last_debug_tick = 0;
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(VisionQueueHandle, &latest, pdMS_TO_TICKS(20)) == pdTRUE)
    {
      float err;
      float err_f;
      TickType_t now_tick;
      int16_t quality;

      // Queue length is 1, this loop is kept for compatibility if size changes later.
      while (xQueueReceive(VisionQueueHandle, &data, 0) == pdTRUE) latest = data;

      err = (float)latest.x * 0.001f;
      err = clampf(err, -1.0f, 1.0f);

      quality = latest.y;
      if (quality < 0) quality = 0;
      if (quality > 1000) quality = 1000;

      now_tick = xTaskGetTickCount();
      taskENTER_CRITICAL();
      g_vision_runtime.err_f += LINE_VISION_ERR_LPF_A * (err - g_vision_runtime.err_f);
      g_vision_runtime.quality = quality;
      g_vision_runtime.tick = now_tick;
      g_vision_runtime.sign = (g_vision_runtime.err_f >= 0.0f) ? 1 : -1;
      err_f = g_vision_runtime.err_f;
      taskEXIT_CRITICAL();

#if LINE_VISION_DEBUG_UART1_EN
      if ((now_tick - last_debug_tick) >= pdMS_TO_TICKS(LINE_VISION_DEBUG_MS))
      {
        char msg[80];
        int n = snprintf(
          msg,
          sizeof(msg),
          "VRAW x=%d y=%d err=%ld q=%d\r\n",
          (int)latest.x,
          (int)latest.y,
          (long)(err_f * 1000.0f),
          (int)quality
        );
        if (n > 0)
        {
          HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)n, 20);
        }
        last_debug_tick = now_tick;
      }
#endif
    }
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
  uint8_t last_mode = g_ucMode;
  uint8_t cmd_char;// 串口/队列收到的指令字符
  SensorSnapshot snap; // 本周期使用的传感器快照
  TickType_t lastWakeTime = xTaskGetTickCount();// vTaskDelayUntil 的参考时间
  TickType_t lastTick = lastWakeTime; // 上一次计算 dt 的时间戳
  float dt; // 逻辑周期（秒）
  g_last_cmd_tick = xTaskGetTickCount(); // 记录最近一次指令的时间戳

  /* Infinite loop */
  for(;;)
  {

    //获取准确dt
    TickType_t now = xTaskGetTickCount();
    dt = (now - lastTick) * 0.001f;
    if (dt <= 0.0f) dt = 0.05f;
    lastTick = now;

    //确保数据传入完整函数
    SensorSnapshot snap1, snap2;
    do {
      snap1 = g_sensor_snapshot;
      snap2 = g_sensor_snapshot;
    } while ((snap1.seq != snap2.seq) || (snap1.seq & 1));
    snap = snap1;

    //蓝牙按键控制
    if (xQueueReceive(CommandQueueHandle, &cmd_char, 0) == pdTRUE)
    {
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
          if(g_ucMode > 4) g_ucMode = 1;
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
 //切换模式清数据
    if (g_ucMode != last_mode)
    {
      PID_Reset(&pidMotor1Speed);
      PID_Reset(&pidMotor2Speed);
      PID_Reset(&pid_pidHW_Tracking);
      PID_Reset(&pidFollow);
      PID_Reset(&mpu6050Movement);
      g_pid_out = 0.0f;
      g_pid_out1 = 0.0f;
      g_pid_out2 = 0.0f;
      g_follow_pid_out = 0.0f;
      g_fMPU6050YawMovePidOut = 0.0f;
      g_fMPU6050YawMovePidOut1 = 0.0f;
      g_fMPU6050YawMovePidOut2 = 0.0f;
      taskENTER_CRITICAL();
      g_vision_runtime.err_f = 0.0f;
      g_vision_runtime.quality = 0;
      g_vision_runtime.tick = 0;
      g_vision_runtime.sign = 1;
      taskEXIT_CRITICAL();
      g_line_vision_state = LINE_VISION_LOST_STOP;
      avoid_state = AVOID_IDLE;
      if (g_ucMode == 4)
      {
        mpu6050Movement.target_val = snap.yaw;
      }
      last_mode = g_ucMode;
    }

    //按键超时
    // if (g_uart_manual_active && (now - g_last_cmd_tick) > pdMS_TO_TICKS(1000))
    // {
    //   motorPidSetSpeed(0, 0);
    //   g_uart_manual_active = 0;
    // }

    //各模块功能
    switch (g_ucMode) {
      case 0:
        if (!g_uart_manual_active) {
          motorPidSetSpeed(0,0);
        }
        break;

      //循迹（未完成）
      case 1:
      {
        TickType_t vision_age;
        TickType_t vision_tick;
        TickType_t now_tick = xTaskGetTickCount();
        float vision_err;
        float base_speed;
        float line_base;
        float line_search;
        float line_min;
        float line_max;
        int16_t vision_quality;
        int8_t vision_sign;

        /* Runtime guard for invalid line parameters loaded from flash. */
        line_base = g_line_base_speed;
        line_search = g_line_search_speed;
        line_min = g_line_min_speed;
        line_max = g_line_max_speed;
        if (line_max < 0.3f) line_max = 4.0f;
        if (line_search < 0.2f) line_search = 1.5f;
        if (line_base < 0.2f) line_base = 2.0f;
        if (line_min > line_max) line_min = 0.5f;

        taskENTER_CRITICAL();
        vision_err = g_vision_runtime.err_f;
        vision_quality = g_vision_runtime.quality;
        vision_tick = g_vision_runtime.tick;
        vision_sign = g_vision_runtime.sign;
        taskEXIT_CRITICAL();

        if (vision_tick == 0)
        {
          /* No frame received yet: keep searching instead of hard stop. */
          g_line_vision_state = LINE_VISION_SEARCH;
        }
        else
        {
          vision_age = now_tick - vision_tick;
          if ((vision_age <= pdMS_TO_TICKS(LINE_VISION_FRESH_MS)) &&
              (vision_quality >= LINE_VISION_QUALITY_TH))
          {
            g_line_vision_state = LINE_VISION_TRACK;
          }
          else if (vision_age <= pdMS_TO_TICKS(LINE_VISION_STOP_MS))
          {
            g_line_vision_state = LINE_VISION_SEARCH;
          }
          else
          {
            g_line_vision_state = LINE_VISION_LOST_STOP;
          }
        }

        if (g_line_vision_state == LINE_VISION_TRACK)
        {
          float track_state = vision_err * 3.0f; // keep close to old 4-sensor scale
          base_speed = line_base - LINE_VISION_SLOW_GAIN * fabsf(vision_err);
          if (base_speed < line_search) base_speed = line_search;

          g_pid_out = PID_realize(&pid_pidHW_Tracking, track_state, dt);
          g_pid_out1 = base_speed + g_pid_out;
          g_pid_out2 = base_speed - g_pid_out;
          g_pid_out1 = clampf(g_pid_out1, line_min, line_max);
          g_pid_out2 = clampf(g_pid_out2, line_min, line_max);
          motorPidSetSpeed(g_pid_out1, g_pid_out2);
        }
        else if (g_line_vision_state == LINE_VISION_SEARCH)
        {
          float turn = LINE_VISION_SEARCH_TURN * (float)vision_sign;
          g_pid_out1 = line_search + turn;
          g_pid_out2 = line_search - turn;
          g_pid_out1 = clampf(g_pid_out1, line_min, line_max);
          g_pid_out2 = clampf(g_pid_out2, line_min, line_max);
          motorPidSetSpeed(g_pid_out1, g_pid_out2);
        }
        else
        {
          g_pid_out = 0.0f;
          g_pid_out1 = 0.0f;
          g_pid_out2 = 0.0f;
          motorPidSetSpeed(0,0);
        }
        break;
      }

      //避障
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
              avoid_set_state(AVOID_IDLE, 200);
            }
            break;
        }
        break;

    //跟随
      case 3:
        if ((snap.sr04 > 0.0f) && (snap.sr04 < 60.0f))
        {
          g_follow_pid_out = PID_realize(&pidFollow, snap.sr04, dt);


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

    //走直线
      case 4:
      {
        float speed_sync;
        float yaw_err = mpu6050Movement.target_val - snap.yaw;
        while (yaw_err > 180.0f) yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;

        if (fabsf(yaw_err) < 1.5f)
        {
          g_fMPU6050YawMovePidOut = 0.0f;
        }
        else
        {
          g_fMPU6050YawMovePidOut = PID_realize(&mpu6050Movement, snap.yaw,dt);
        }

        if (g_fMPU6050YawMovePidOut > 0.6f) g_fMPU6050YawMovePidOut = 0.6f;
        if (g_fMPU6050YawMovePidOut < -0.6f) g_fMPU6050YawMovePidOut = -0.6f;

//        speed_sync = 0.15f * (Motor2Speed - Motor1Speed);
//        if (speed_sync > 0.35f) speed_sync = 0.35f;
//        if (speed_sync < -0.35f) speed_sync = -0.35f;

//        g_fMPU6050YawMovePidOut1 = 1.5f + g_fMPU6050YawMovePidOut + speed_sync;
        g_fMPU6050YawMovePidOut1 = 1.5f + g_fMPU6050YawMovePidOut;

//        g_fMPU6050YawMovePidOut2 = 1.5f - g_fMPU6050YawMovePidOut - speed_sync;
        g_fMPU6050YawMovePidOut2 = 1.5f - g_fMPU6050YawMovePidOut;

        if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;
        if(g_fMPU6050YawMovePidOut1 < 0 ) g_fMPU6050YawMovePidOut1 =0;
        if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
        if(g_fMPU6050YawMovePidOut2 < 0) g_fMPU6050YawMovePidOut2 = 0;

        motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
        break;
      }
      case 5:
        motorPidSetSpeed(0,0);
        break;
      default:
        g_ucMode = 0;
        motorPidSetSpeed(0,0);
        break;
    }
    //绝对延时
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));
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
//ui界面
void StartTask06(void const * argument)
{
    /* USER CODE BEGIN StartTask06 */
  OLED_Init();
  OLED_Clear();
  // KEY1: Up/Enter, KEY2: Down/Back (short/long press).
  //按键状态
  KeyState key_up = {0};
  KeyState key_down = {0};
  //菜单状态
  UiView view = UI_VIEW_MENU;
  int8_t current_parent = MENU_IDX_ROOT;
  uint8_t cursor = 0;
  uint8_t view_offset = 0;
  //储存旧值，退回上一值
  uint8_t edit_index = 0;
  float edit_backup_f = 0.0f;
  int32_t edit_backup_i = 0;
  //启动信号
  uint32_t last_draw = 0;
  uint8_t redraw_pending = 1;

  for(;;)
  {
    //获取时间
    uint32_t now = HAL_GetTick();
    //储存长按还是短按
    KeyEvent up_evt = {0};
    KeyEvent down_evt = {0};
    uint8_t need_redraw = 0;

    //查看高低电平
    uint8_t raw_up = (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET);
    uint8_t raw_down = (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET);
    ui_key_update(&key_up, raw_up, now, &up_evt);
    ui_key_update(&key_down, raw_down, now, &down_evt);

    if (view == UI_VIEW_MENU)
    {
      uint8_t count = ui_menu_child_count(current_parent);
      if (count == 0)
      {
        cursor = 0;
        view_offset = 0;
        if (down_evt.long_press && current_parent != MENU_IDX_ROOT)
        {
          uint8_t child = (uint8_t)current_parent;
          current_parent = g_menu_items[current_parent].parent;
          cursor = ui_menu_child_pos(current_parent, child);
          view_offset = 0;
          if (cursor >= UI_LIST_LINES) view_offset = cursor - (UI_LIST_LINES - 1);
          need_redraw = 1;
        }
      }
      //正常短按
      else
      {
        if (cursor >= count) cursor = (uint8_t)(count - 1);

        if (up_evt.short_press)
        {
          cursor = (cursor == 0) ? (uint8_t)(count - 1) : (uint8_t)(cursor - 1);
          need_redraw = 1;
        }
        if (down_evt.short_press)
        {
          cursor++;
          if (cursor >= count) cursor = 0;
          need_redraw = 1;
        }
      //翻页
        if (cursor < view_offset) view_offset = cursor;
        if (cursor >= view_offset + UI_LIST_LINES) view_offset = cursor - (UI_LIST_LINES - 1);

        if (up_evt.long_press)
        {
          int16_t idx = ui_menu_child_index(current_parent, cursor);
          if (idx >= 0)
          {
            const MenuItem *item = &g_menu_items[idx];
            if (item->type == MENU_TYPE_SUBMENU)
            {
              current_parent = (int8_t)idx;
              cursor = 0;
              view_offset = 0;
              need_redraw = 1;
            }
            else if (item->type == MENU_TYPE_PARAM)
            {
              view = UI_VIEW_EDIT;
              redraw_pending = 1;
              edit_index = (uint8_t)idx;
              if (item->value_type == MENU_VALUE_FLOAT && item->value_ptr)
              {
                edit_backup_f = *(float *)item->value_ptr;
              }
              else
              {
                edit_backup_i = ui_get_int(item);
              }
              last_draw = 0;
            }
            else if (item->type == MENU_TYPE_ACTION)
            {
              if (item->action_id == MENU_ACTION_STATUS) view = UI_VIEW_STATUS;
              redraw_pending = 1;
              last_draw = 0;
            }
          }
        }

        if (down_evt.long_press)
        {
          if (current_parent != MENU_IDX_ROOT)
          {
            uint8_t child = (uint8_t)current_parent;
            current_parent = g_menu_items[current_parent].parent;
            cursor = ui_menu_child_pos(current_parent, child);
            view_offset = 0;
            if (cursor >= UI_LIST_LINES) view_offset = cursor - (UI_LIST_LINES - 1);
            need_redraw = 1;
          }
        }
      }

      if (view == UI_VIEW_MENU && (need_redraw || redraw_pending))
      {
        ui_draw_menu(current_parent, cursor, view_offset, 0, edit_index);
        last_draw = now;
        redraw_pending = 0;
      }
    }
    //改数据
    else if (view == UI_VIEW_EDIT)
    {
      const MenuItem *item = &g_menu_items[edit_index];
      if (up_evt.short_press || down_evt.short_press)
      {
        if (item->value_type == MENU_VALUE_FLOAT && item->value_ptr)
        {
          float v = *(float *)item->value_ptr;
          float step = item->step;
          if (step == 0.0f) step = 0.1f;
          if (up_evt.short_press) v += step;
          if (down_evt.short_press) v -= step;
          if (v > item->max) v = item->max;
          if (v < item->min) v = item->min;
          *(float *)item->value_ptr = v;
        }
        else
        {
          int32_t v = ui_get_int(item);
          int32_t step = (int32_t)item->step;
          if (step <= 0) step = 1;
          if (up_evt.short_press) v += step;
          if (down_evt.short_press) v -= step;
          if (v > (int32_t)item->max) v = (int32_t)item->max;
          if (v < (int32_t)item->min) v = (int32_t)item->min;
          ui_set_int(item, v);
        }
        need_redraw = 1;
      }

      if (up_evt.long_press)
      {
        uint8_t changed = 0;
        if (item->value_type == MENU_VALUE_FLOAT && item->value_ptr)
        {
          float now_v = *(float *)item->value_ptr;
          changed = (fabsf(now_v - edit_backup_f) > 0.0001f) ? 1U : 0U;
        }
        else
        {
          changed = (ui_get_int(item) != edit_backup_i) ? 1U : 0U;
        }
        if (changed)
        {
          settings_mark_dirty();
        }
        view = UI_VIEW_MENU;
        redraw_pending = 1;
        last_draw = 0;
      }
      if (down_evt.long_press)
      {
        if (item->value_type == MENU_VALUE_FLOAT && item->value_ptr)
        {
          *(float *)item->value_ptr = edit_backup_f;
        }
        else
        {
          ui_set_int(item, edit_backup_i);
        }
        view = UI_VIEW_MENU;
        redraw_pending = 1;
        last_draw = 0;
      }

      if (view == UI_VIEW_EDIT && (need_redraw || redraw_pending))
      {
        ui_draw_menu(current_parent, cursor, view_offset, 1, edit_index);
        last_draw = now;
        redraw_pending = 0;
      }
    }
    else if (view == UI_VIEW_STATUS)
    {
      if (up_evt.long_press || down_evt.long_press)
      {
        view = UI_VIEW_MENU;
        redraw_pending = 1;
        last_draw = 0;
      }
      if (view == UI_VIEW_STATUS && (need_redraw || redraw_pending || (now - last_draw) > UI_STATUS_REFRESH_MS))
      {
        ui_draw_status();
        last_draw = now;
        redraw_pending = 0;
      }
    }

    osDelay(5);
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

//vofa
// static void vofa_send_waveform(float target_speed, float actual_speed)
// {
//   char tx[64];
//   long target_milli = (long)(target_speed * 1000.0f);
//   long actual_milli = (long)(actual_speed * 1000.0f);
//   int n = snprintf(tx, sizeof(tx), "speed:%ld,%ld\r\n", target_milli, actual_milli);
//   if (n > 0)
//   {
//     size_t len = strlen(tx);
//     HAL_UART_Transmit(&huart1, (uint8_t *)tx, (uint16_t)len, 5);
//   }
// }
//万能数据读写助手，UI 逻辑都统一按 32 位处理
static float clampf(float v, float vmin, float vmax)
{
  if (v < vmin) return vmin;
  if (v > vmax) return vmax;
  return v;
}

static int32_t ui_get_int(const MenuItem *item)
{
  if (item == 0 || item->value_ptr == 0) return 0;
  if (item->value_type == MENU_VALUE_INT8) return *(int8_t *)item->value_ptr;
  if (item->value_type == MENU_VALUE_INT16) return *(int16_t *)item->value_ptr;
  if (item->value_type == MENU_VALUE_INT32) return *(int32_t *)item->value_ptr;
  return 0;
}
static void ui_set_int(const MenuItem *item, int32_t v)
{
  if (item == 0 || item->value_ptr == 0) return;
  if (item->value_type == MENU_VALUE_INT8) *(int8_t *)item->value_ptr = (int8_t)v;
  else if (item->value_type == MENU_VALUE_INT16) *(int16_t *)item->value_ptr = (int16_t)v;
  else if (item->value_type == MENU_VALUE_INT32) *(int32_t *)item->value_ptr = (int32_t)v;
}

//将历史的放在k里，更新evt
static void ui_key_update(KeyState *k, uint8_t raw, uint32_t now, KeyEvent *evt)
{
  evt->short_press = 0;
  evt->long_press = 0;

  if (raw != k->last_raw)
  {
    k->last_raw = raw;
    k->last_change = now;
  }
  else if ((now - k->last_change) >= UI_KEY_DEBOUNCE_MS && raw != k->stable)
  {
    k->stable = raw;
    if (k->stable)
    {
      k->press_tick = now;
      k->long_sent = 0;
    }
    else
    {
      if (!k->long_sent) evt->short_press = 1;
    }
  }

  if (k->stable && !k->long_sent && (now - k->press_tick) >= UI_KEY_LONG_MS)
  {
    evt->long_press = 1;
    k->long_sent = 1;
  }
}


static void ui_format_value(const MenuItem *item, char *buf, uint8_t len)
{
  if (item->value_type == MENU_VALUE_FLOAT && item->value_ptr)
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

//菜单层级搜索算法
static uint8_t ui_menu_child_count(int8_t parent)
{
  uint8_t count = 0;
  uint8_t i;
  for (i = 0; i < g_menu_count; i++)
  {
    if (g_menu_items[i].parent == parent) count++;
  }
  return count;
}
static int16_t ui_menu_child_index(int8_t parent, uint8_t child_pos)
{
  uint8_t count = 0;
  uint8_t i;
  for (i = 0; i < g_menu_count; i++)
  {
    if (g_menu_items[i].parent != parent) continue;
    if (count == child_pos) return (int16_t)i;
    count++;
  }
  return -1;
}
static uint8_t ui_menu_child_pos(int8_t parent, uint8_t child_index)
{
  uint8_t count = 0;
  uint8_t i;
  for (i = 0; i < g_menu_count; i++)
  {
    if (g_menu_items[i].parent != parent) continue;
    if (i == child_index) return count;
    count++;
  }
  return 0;
}

//绘图引擎
static void ui_draw_line(uint8_t row, const char *text)
{
  char buf[UI_LINE_CHARS + 1];
  uint8_t i = 0;

  if (text != 0)
  {
    for (; i < UI_LINE_CHARS && text[i] != '\0'; i++) buf[i] = text[i];
  }
  for (; i < UI_LINE_CHARS; i++) buf[i] = ' ';
  buf[UI_LINE_CHARS] = '\0';

  OLED_ShowString(0, row, (uint8_t *)buf, 12);
}
static void ui_draw_menu(int8_t parent, uint8_t cursor, uint8_t view_offset, uint8_t edit_mode, uint8_t edit_index)
{
  uint8_t count = ui_menu_child_count(parent);
  uint8_t line;
  int16_t cursor_idx = ui_menu_child_index(parent, cursor);

  if (count == 0)
  {
    ui_draw_line(0, ">Empty");
    ui_draw_line(1, "");
    ui_draw_line(2, "");
    ui_draw_line(3, "");
    return;
  }
  for (line = 0; line < UI_LIST_LINES; line++)
  {
    uint8_t pos = (uint8_t)(view_offset + line);
    int16_t idx;
    const MenuItem *item;
    char linebuf[24];
    char valbuf[12];
    char mark = ' ';

    if (pos >= count) break;
    idx = ui_menu_child_index(parent, pos);
    if (idx < 0) break;

    item = &g_menu_items[idx];
    if (idx == cursor_idx) mark = '>';
    if (edit_mode && idx == edit_index) mark = '*';

    if (item->type == MENU_TYPE_PARAM)
    {
      ui_format_value(item, valbuf, sizeof(valbuf));
      snprintf(linebuf, sizeof(linebuf), "%c%s:%s", mark, item->name, valbuf);
    }
    else if (item->type == MENU_TYPE_SUBMENU)
    {
      snprintf(linebuf, sizeof(linebuf), "%c%s>", mark, item->name);
    }
    else
    {
      snprintf(linebuf, sizeof(linebuf), "%c%s", mark, item->name);
    }

    ui_draw_line(line, linebuf);
  }

  for (; line < UI_LIST_LINES; line++) ui_draw_line(line, "");
}
static void ui_draw_status(void)
{
  char line[32];
  snprintf(line, sizeof(line), "Mode:%d", g_ucMode);
  ui_draw_line(0, line);
  snprintf(line, sizeof(line), "L:%.1f R:%.1f", Motor1Speed, Motor2Speed);
  ui_draw_line(1, line);
  snprintf(line, sizeof(line), "D:%.1f M:%.1f", dist, mile);
  ui_draw_line(2, line);
  snprintf(line, sizeof(line), "Yaw:%.1f", yaw);
  ui_draw_line(3, line);
}



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















