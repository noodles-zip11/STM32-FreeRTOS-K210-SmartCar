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

/*
 * 本文件是整车“运行时核心”：
 * 1) 创建 FreeRTOS 任务/队列（静态分配）；
 * 2) 维护传感器快照、视觉运行态、模式状态机；
 * 3) 在 LogicTask 中把“模式逻辑”转换为目标速度；
 * 4) 在 ControlTask 中把目标速度闭环成 PWM 输出；
 * 5) 提供本地 OLED UI 参数菜单。
 *
 * 阅读建议：
 * - 先看 MX_FREERTOS_Init()（任务/队列拓扑）
 * - 再看 StartSensorTask()/StartVisionTask()（数据来源）
 * - 再看 StartLogicTask()（行为决策）
 * - 最后看 StartControlTask()（执行层闭环）
 */

//pid变量
/*
 * 巡线模式参数（同时也是 UI 可调参数，会被 settings 模块持久化）。
 * 放在全局是因为逻辑任务/UI/Flash 保存都会访问。
 */
float g_line_base_speed = 1.2f;
float g_line_search_speed = 0.8f;
float g_line_max_speed = 2.0f;
float g_line_min_speed = 0.5f;

//电机测速和设置速度
/*
 * 编码器计数与速度反馈：
 * EncoderXCount 是单个控制周期内的增量计数；
 * MotorXSpeed 是换算后的实际速度（给 PID 和状态显示使用）。
 */
short Encoder1Count =0 ;
short Encoder2Count =0 ;
float Motor1Speed = 0.00;
float Motor2Speed = 0.00;
extern tpid pidMotor1Speed;
extern tpid pidMotor2Speed;

//里程
float mile =0.00 ;

//串口手动控制超时
/* 串口手动控制状态：用于区分“手动命令”与“自动模式逻辑”。 */
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
float g_line_turn_cmd_f = 0.0f;
uint8_t g_line_corner_active = 0U;
float g_line_last_track_speed = 0.0f;
float g_line_last_track_turn = 0.0f;
TickType_t g_line_search_enter_tick = 0U;
int8_t g_line_search_latched_sign = 1;

/*
 * 视觉巡线调参常量（单位约定）：
 * - *_MS：时间窗口（毫秒）
 * - *_TH：阈值（质量/误差）
 * - *_SPEED_*：目标轮速（与 PID 目标速度同单位）
 * - *_TURN_*：左右轮差速转向量（与速度同单位）
 *
 * 这些常量主要影响 Mode1 的三个子状态：TRACK / SEARCH / LOST_STOP。
 */
#define LINE_VISION_FRESH_MS      150U
#define LINE_VISION_STOP_MS       800U
#define LINE_VISION_QUALITY_TH    350
#define LINE_VISION_EDGE_TRACK_QUALITY_TH 140
#define LINE_VISION_EDGE_TRACK_ERR_TH     0.55f
#define LINE_VISION_ERR_LPF_A     0.15f
#define LINE_VISION_CENTER_DEADBAND 0.05f
#define LINE_VISION_TRACK_SCALE   1.2f
#define LINE_VISION_SLOW_GAIN      0.80f
#define LINE_VISION_SEARCH_TURN    0.25f
#define LINE_VISION_TURN_SLOW_GAIN 0.60f
#define LINE_VISION_TURN_CAP_ABS   0.32f
#define LINE_VISION_TURN_CAP_RATIO 0.36f
#define LINE_VISION_TURN_SLEW_STEP 0.05f
/* 临时硬限速：巡线模式优先稳定性，其次才是速度。 */
#define LINE_VISION_TRACK_WHEEL_MAX_CAP   1.15f
#define LINE_VISION_SEARCH_SPEED_CAP      0.80f
#define LINE_VISION_CORNER_ENTER_ERR      0.26f
#define LINE_VISION_CORNER_EXIT_ERR       0.14f
#define LINE_VISION_CORNER_INNER_MIN      0.0f
#define LINE_VISION_CORNER_EXTRA_TURN_CAP 0.20f
#define LINE_VISION_CORNER_EXTRA_TURN_RATIO 0.22f
#define LINE_VISION_CORNER_EXTRA_SLEW     0.05f
#define LINE_VISION_SEARCH_CORNER_TURN    0.48f
#define LINE_VISION_SEARCH_KEEP_SPEED_MS   220U
#define LINE_VISION_SEARCH_KEEP_TURN_TH    0.06f
/* 临时保守 PID 预设（进入 Mode 1 巡线模式时应用）。 */
#define LINE_VISION_FORCE_STABLE_PID_KP   (-1.2f)
#define LINE_VISION_FORCE_STABLE_PID_KI   (0.0f)
#define LINE_VISION_FORCE_STABLE_PID_KD   (0.0f)
//蓝牙接收
uint8_t g_ucusrtrecivedate;

//模式切换
/*
 * 模式编号约定（由 LogicTask 的 switch(g_ucMode) 实现）：
 * 0=待机/人工串口控制保持
 * 1=视觉巡线
 * 2=超声避障
 * 3=距离跟随
 * 4=IMU 航向保持直行
 * 5=保留/停车
 */
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
/*
 * 传感器快照（SensorTask 写，LogicTask 读）。
 * seq 使用“前后自增”的方式，读线程可通过双读检查拿到一致快照。
 */
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
  /* 统一设置避障状态和状态持续时间，避免重复写 tick 换算。 */
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
  TickType_t good_tick;
  int8_t sign;
} VisionRuntime;

/*
 * 视觉运行态由 VisionTask 更新，LogicTask 读取。
 * sign 用于丢线后按“上次偏差方向”继续搜索，避免来回乱转。
 */
static volatile VisionRuntime g_vision_runtime = {0.0f, 0, 0, 0, 1};
static LineVisionState g_line_vision_state = LINE_VISION_LOST_STOP;

//准备队列静态内存和创建静态队列
/*
 * 静态队列/任务内存：
 * 使用静态分配能让 RAM 占用更可控，减少运行时动态分配失败风险。
 */
static StaticQueue_t xCommandQueueBuffer;
static uint8_t ucCommandQueueStorage[10 * sizeof(uint8_t)];
static StaticQueue_t xVisionQueueBuffer;
static uint8_t ucVisionQueueStorage[1 * sizeof(VisionData_t)];
static StaticQueue_t xMotorTargetQueueBuffer;
static uint8_t ucMotorTargetQueueStorage[1 * sizeof(MotorTarget_t)];

//准备任务静态内存和TCB
/* 各任务的静态 TCB/栈空间。栈大小是按任务负载手工分配的。 */
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
/* 128px OLED + 8px 等宽字体 => 一行安全显示 16 个字符。 */
#define UI_LINE_CHARS 16
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
  /* 菜单项名称（显示在 OLED 上）。 */
  const char *name;
  /* 菜单项类型：子菜单 / 参数 / 动作页。 */
  MenuItemType type;
  /* 父节点索引；根节点为 -1。 */
  int8_t parent;
  /* 参数值类型（UI 负责按类型格式化/读写）。 */
  MenuValueType value_type;
  /* 指向实际参数变量（float/int8/int16/int32）。 */
  void *value_ptr;
  /* 调整步进（编辑模式短按一次的增减量）。 */
  float step;
  /* 参数最小/最大值（编辑时限幅）。 */
  float min;
  float max;
  /* 动作项的附加 ID（例如进入状态页）。 */
  uint8_t action_id;
} MenuItem;

typedef struct {
  /* 去抖后稳定电平（1=按下，0=松开）。 */
  uint8_t stable;
  /* 原始采样电平（未去抖）。 */
  uint8_t last_raw;
  /* 原始电平最近一次变化时刻。 */
  uint32_t last_change;
  /* 确认按下后的时间戳（用于判定长按）。 */
  uint32_t press_tick;
  /* 长按事件是否已经发出（防止重复触发）。 */
  uint8_t long_sent;
} KeyState;

typedef struct {
  /* 短按：按下并释放，且未触发过长按。 */
  uint8_t short_press;
  /* 长按：按住超过阈值，仅触发一次。 */
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
/*
 * 菜单表采用“平铺数组 + parent 索引”而不是树指针：
 * - 不需要动态分配；
 * - 更适合在 MCU 上做静态初始化；
 * - 便于 settings/UI 共享参数指针。
 */
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
static void ui_format_float_fixed(float v, uint8_t decimals, char *buf, uint8_t len);
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

/* GetIdleTaskMemory 函数声明（用于 FreeRTOS 静态内存分配支持） */
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
  /*
   * 三个关键队列：
   * 1) CommandQueue：UART3 命令字节（异步输入）
   * 2) VisionQueue：视觉最新数据（长度 1，覆盖旧值）
   * 3) MotorTargetQueue：目标速度（长度 1，控制任务只取最新）
   */
  CommandQueueHandle = xQueueCreateStatic(10, sizeof(uint8_t),
                                          ucCommandQueueStorage, &xCommandQueueBuffer);
  VisionQueueHandle = xQueueCreateStatic(1, sizeof(VisionData_t),
                                         ucVisionQueueStorage, &xVisionQueueBuffer);
  MotorTargetQueueHandle = xQueueCreateStatic(1, sizeof(MotorTarget_t),
                                              ucMotorTargetQueueStorage, &xMotorTargetQueueBuffer);
  /* 启动阶段队列创建失败必须立刻暴露，避免后续空句柄运行。 */
  configASSERT(CommandQueueHandle != NULL);
  configASSERT(VisionQueueHandle != NULL);
  configASSERT(MotorTargetQueueHandle != NULL);

  /*
   * 任务职责分层：
   * - ControlTask：电机速度闭环（高频、优先级高）
   * - SensorTask/VisionTask：采集并做轻量预处理
   * - LogicTask：模式切换与行为决策
   * - StartUITask：本地菜单与参数编辑
   * - DefaultTask：后台维护（如延迟保存参数）
   */
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
  /*
   * UI 任务创建时先给低优先级，确认系统能拉起后再升到 Normal。
   * 这样更容易在调试阶段观察是否因 UI 栈或初始化阻塞影响系统启动。
   */
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
  /*
   * 后台维护任务：
   * 当前主要负责 settings 的延迟保存，避免在 UI 按键过程中频繁写 Flash。
   */
  /* Infinite loop */

  for(;;)
  {
    /* 只做短小后台工作，保持任务简单稳定。 */
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
  /*
   * 电机速度闭环核心任务（10ms）：
   * 输入：目标速度队列 + 编码器增量
   * 输出：左右轮 PWM（通过 Motor_Set）
   * 使用 vTaskDelayUntil 保持控制节拍稳定。
   */
  TickType_t lastWakeTime = xTaskGetTickCount();// vTaskDelayUntil 使用的参考时间
  TickType_t lastTick = lastWakeTime; // 上一次计算 dt 的时间戳
  MotorTarget_t target;  // 队列里接收到的目标速度
  float dt;  // 控制周期（秒）

  static float m1_speed_f = 0.0f; // 电机1速度低通滤波后的值
  static float m2_speed_f = 0.0f; // 电机2速度低通滤波后的值
  static float last_target_m1 = 0.0f;
  static float last_target_m2 = 0.0f;
  /* 编码器方向符号由接线/机械安装决定；如有需要可在这里调整。 */
  static int8_t m1_enc_sign = 1;
  static int8_t m2_enc_sign = -1;



  // 开启编码器
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  /* Infinite loop */
  for(;;)
  {
    // 1) 获取最新目标速度（来自逻辑任务/串口手动命令）
    //    队列长度为 1，拿到的永远是最新值。
    if (xQueueReceive(MotorTargetQueueHandle, &target, 0) == pdTRUE)
    {
      float new_target_m1 = target.left;
      float new_target_m2 = target.right;

      /* 目标方向反转时清 PID 状态，避免旧积分导致瞬间过冲。 */
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

    // 2) 计算本周期真实 dt（秒），而不是假设恒定 10ms
    /* 用实际 tick 差计算 dt，避免任务抖动让速度估算偏掉。 */
    TickType_t now = xTaskGetTickCount();
    dt = (now - lastTick) * 0.001f;
    if (dt <= 0.0f) dt = 0.01f;
    lastTick = now;

    // 3) 读取编码器增量并清零（增量式测速）
    /* 读取本周期编码器增量后清零，下一周期重新累计。 */
    Encoder1Count=(short)__HAL_TIM_GET_COUNTER(&htim4);
    Encoder2Count=(short)__HAL_TIM_GET_COUNTER(&htim2);

    __HAL_TIM_SET_COUNTER(&htim4,0);
    __HAL_TIM_SET_COUNTER(&htim2,0);

    // 4) 把编码器计数换算成轮速（圈/秒），再按接线方向修正符号
    float rev1 = (float)Encoder1Count / (ENC_PPR * GEAR_RATIO);
    float rev2 = (float)Encoder2Count / (ENC_PPR * GEAR_RATIO);
    float raw_m1_speed = rev1 / dt;
    float raw_m2_speed = rev2 / dt;

    Motor1Speed = (float)m1_enc_sign * raw_m1_speed;
    Motor2Speed = (float)m2_enc_sign * raw_m2_speed;

    /* 先对速度做低通，能显著减小编码器抖动对 PID（尤其 D 项）的影响。 */
    m1_speed_f += SPEED_LPF_A * (Motor1Speed - m1_speed_f);
    m2_speed_f += SPEED_LPF_A * (Motor2Speed - m2_speed_f);

    // 5) 速度低通后进入 PID，输出差速 PWM
    Motor_Set(
      PID_realize(&pidMotor1Speed, m1_speed_f, dt),
      PID_realize(&pidMotor2Speed, m2_speed_f, dt)
    );

    // 6) 调试波形发送（当前默认关闭，避免占用串口带宽）
    // if ((now - g_vofa_tx_tick) >= pdMS_TO_TICKS(50))
    // {
    //   float target_speed = 0.5f * (pidMotor1Speed.target_val + pidMotor2Speed.target_val);
    //   float actual_speed = 0.5f * (m1_speed_f + m2_speed_f);
    //   vofa_send_waveform(target_speed, actual_speed);
    //   g_vofa_tx_tick = now;
    // }

    // 7) 控制任务是系统关键节拍之一，用它喂狗可尽早暴露调度卡死
    HAL_IWDG_Refresh(&hiwdg);  // 周期性刷新 IWDG，避免任务阻塞导致系统复位

    // 8) 使用绝对延时维持固定控制节拍
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
  /*
   * 传感器采样任务（20ms）：
   * 统一采集灰度、超声波、IMU，并整理成一份快照给逻辑任务使用。
   * 好处是逻辑层不用关心每个驱动的调用顺序和耗时。
   */

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
    /*
     * 使用 seq 前后自增形成“写入窗口”：
     * 读取端如果看到 seq 变化或奇数，说明读到了半更新数据，会重读。
     */
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
  /*
   * 视觉任务职责很克制：
   * - 从队列取 ISR 已拆包的数据
   * - 做误差缩放/质量裁剪/低通
   * - 更新供逻辑任务使用的视觉运行态
   * 不在这里直接控车，避免视觉延迟影响整体调度。
   */
  /* 在这里启动 UART2 接收，避免 K210 数据在 main() 初始化阶段灌入。 */
  HAL_UART_Receive_IT(&huart2, &RxBuffer, 1);
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(VisionQueueHandle, &latest, pdMS_TO_TICKS(20)) == pdTRUE)
    {
      float err;
      TickType_t now_tick;
      int16_t quality;

      // 当前队列长度为 1；保留该循环是为了兼容后续可能调整队列长度。
      while (xQueueReceive(VisionQueueHandle, &data, 0) == pdTRUE) latest = data;

      /* 把视觉横向偏差缩放到较小范围，后面控制逻辑更容易调参。 */
      /* K210 的 x 符号方向与小车转向约定相反，这里统一取反一次。 */
      err = -(float)latest.x * 0.001f;
      err = clampf(err, -1.0f, 1.0f);

      quality = latest.y;
      if (quality < 0) quality = 0;
      if (quality > 1000) quality = 1000;

      now_tick = xTaskGetTickCount();
      /* 一次临界区内更新整组视觉状态，减少逻辑任务读到“半更新状态”的概率。 */
      taskENTER_CRITICAL();
      g_vision_runtime.err_f += LINE_VISION_ERR_LPF_A * (err - g_vision_runtime.err_f);
      g_vision_runtime.quality = quality;
      g_vision_runtime.tick = now_tick;
      if ((quality >= LINE_VISION_QUALITY_TH) ||
          ((quality >= LINE_VISION_EDGE_TRACK_QUALITY_TH) &&
           (fabsf(g_vision_runtime.err_f) >= LINE_VISION_EDGE_TRACK_ERR_TH)))
      {
        g_vision_runtime.good_tick = now_tick;
        g_vision_runtime.sign = (g_vision_runtime.err_f >= 0.0f) ? 1 : -1;
      }
      taskEXIT_CRITICAL();

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
  /*
   * 逻辑任务（20ms）负责“做决定”，不负责底层闭环：
   * - 收命令、切模式
   * - 读取传感器快照
   * - 根据模式生成目标速度（再交给 ControlTask 做速度环）
   */
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
    /*
     * 双读快照确保拿到一份完整数据（与 SensorTask 的 seq 机制配套）。
     * 这样逻辑任务不会读到“灰度是新数据、yaw 还是旧数据”的混合状态。
     */
    SensorSnapshot snap1, snap2;
    do {
      snap1 = g_sensor_snapshot;
      snap2 = g_sensor_snapshot;
    } while ((snap1.seq != snap2.seq) || (snap1.seq & 1));
    snap = snap1;

    //蓝牙按键控制
    if (xQueueReceive(CommandQueueHandle, &cmd_char, 0) == pdTRUE)
    {
      /* 串口命令作为高优先级人工干预入口，收到后先标记为手动模式活跃。 */
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
      /*
       * 模式切换时清所有控制器状态而不是只改 g_ucMode：
       * 防止上一个模式残留的积分项/滤波状态带到新模式里造成突变。
       */
      PID_Reset(&pidMotor1Speed);
      PID_Reset(&pidMotor2Speed);
      PID_Reset(&pid_pidHW_Tracking);
      PID_Reset(&pidFollow);
      PID_Reset(&mpu6050Movement);
      g_pid_out = 0.0f;
      g_pid_out1 = 0.0f;
      g_pid_out2 = 0.0f;
      g_line_turn_cmd_f = 0.0f;
      g_line_corner_active = 0U;
      g_line_last_track_speed = 0.0f;
      g_line_last_track_turn = 0.0f;
      g_line_search_enter_tick = 0U;
      g_line_search_latched_sign = 1;
      g_follow_pid_out = 0.0f;
      g_fMPU6050YawMovePidOut = 0.0f;
      g_fMPU6050YawMovePidOut1 = 0.0f;
      g_fMPU6050YawMovePidOut2 = 0.0f;
      taskENTER_CRITICAL();
      g_vision_runtime.err_f = 0.0f;
      g_vision_runtime.quality = 0;
      g_vision_runtime.tick = 0;
      g_vision_runtime.good_tick = 0;
      g_vision_runtime.sign = 1;
      taskEXIT_CRITICAL();
      g_line_vision_state = LINE_VISION_LOST_STOP;
      avoid_state = AVOID_IDLE;
      if (g_ucMode == 1)
      {
        /* 不在模式切换时强制覆盖循迹 PID，保留用户菜单调参与 Flash 恢复结果。 */
      }
      else if (g_ucMode == 4)
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

    // 各模块功能
    /*
     * 各模式统一输出“目标速度”，真正 PWM 由 ControlTask 闭环计算。
     * 这样模式逻辑和电机控制解耦，调试会清楚很多。
     */
    switch (g_ucMode) {
      case 0:
        /* 待机模式：如果当前没有串口手动控制，就保持停车。 */
        if (!g_uart_manual_active) {
          motorPidSetSpeed(0,0);
        }
        break;

      //循迹（未完成）
      case 1:
      {
        /*
         * 视觉巡线模式（状态机）：
         * TRACK      有新鲜且质量足够的视觉数据，正常跟踪
         * SEARCH     刚丢线但还有最近有效方向，按方向小幅搜索
         * LOST_STOP  长时间无有效线，直接停车更安全
         */
        TickType_t vision_age;
        TickType_t good_age;
        TickType_t vision_tick;
        TickType_t vision_good_tick;
        TickType_t now_tick = xTaskGetTickCount();
        uint8_t prev_line_vision_state = g_line_vision_state;
        float vision_err;
        float base_speed;
        int16_t vision_quality;
        int8_t vision_sign;

        /* 先把视觉运行态拷到局部变量，后续判断就不用长时间占用临界区。 */
        taskENTER_CRITICAL();
        vision_err = g_vision_runtime.err_f;
        vision_quality = g_vision_runtime.quality;
        vision_tick = g_vision_runtime.tick;
        vision_good_tick = g_vision_runtime.good_tick;
        vision_sign = g_vision_runtime.sign;
        taskEXIT_CRITICAL();

        if (vision_tick == 0)
        {
          vision_age = pdMS_TO_TICKS(LINE_VISION_STOP_MS + 1U);
        }
        else
        {
          vision_age = now_tick - vision_tick;
        }

        if (vision_good_tick == 0)
        {
          good_age = pdMS_TO_TICKS(LINE_VISION_STOP_MS + 1U);
        }
        else
        {
          good_age = now_tick - vision_good_tick;
        }

        if ((vision_age <= pdMS_TO_TICKS(LINE_VISION_FRESH_MS)) &&
            ((vision_quality >= LINE_VISION_QUALITY_TH) ||
             ((vision_quality >= LINE_VISION_EDGE_TRACK_QUALITY_TH) &&
              (fabsf(vision_err) >= LINE_VISION_EDGE_TRACK_ERR_TH))))
        {
          g_line_vision_state = LINE_VISION_TRACK;
        }
        else if (good_age <= pdMS_TO_TICKS(LINE_VISION_STOP_MS))
        {
          g_line_vision_state = LINE_VISION_SEARCH;
        }
        else
        {
          g_line_vision_state = LINE_VISION_LOST_STOP;
        }

        if ((g_line_vision_state == LINE_VISION_SEARCH) && (prev_line_vision_state != LINE_VISION_SEARCH))
        {
          g_line_search_enter_tick = now_tick;
          if (vision_sign != 0) g_line_search_latched_sign = vision_sign;
        }
        else if (g_line_vision_state == LINE_VISION_TRACK)
        {
          if (vision_sign != 0) g_line_search_latched_sign = vision_sign;
        }

        if (g_line_vision_state == LINE_VISION_TRACK)
        {
          /*
           * 有效跟踪时转弯越大，基础速度越低：
           * 这样急弯时先降速，能减少冲出赛道/线路的概率。
           */
          float line_wheel_max = fminf(g_line_max_speed, LINE_VISION_TRACK_WHEEL_MAX_CAP);
          float line_search_floor;
          float err_mag;
          float track_output_min;
          if (line_wheel_max < g_line_min_speed) line_wheel_max = g_line_min_speed;

          float track_state;
          if (fabsf(vision_err) < LINE_VISION_CENTER_DEADBAND) vision_err = 0.0f;
          err_mag = fabsf(vision_err);
          if (!g_line_corner_active && (err_mag >= LINE_VISION_CORNER_ENTER_ERR))
          {
            g_line_corner_active = 1U;
          }
          else if (g_line_corner_active && (err_mag <= LINE_VISION_CORNER_EXIT_ERR))
          {
            g_line_corner_active = 0U;
          }
          track_output_min = g_line_corner_active ? LINE_VISION_CORNER_INNER_MIN : g_line_min_speed;
          if (track_output_min < 0.0f) track_output_min = 0.0f;
          line_search_floor = g_line_corner_active ? fminf(0.35f, line_wheel_max) : g_line_min_speed;
          if (line_search_floor < track_output_min) line_search_floor = track_output_min;
          {
            float track_gain = LINE_VISION_TRACK_SCALE + 1.2f * err_mag;
            if (g_line_corner_active) track_gain += 0.35f;
            if (track_gain > 2.2f) track_gain = 2.2f;
            track_state = vision_err * track_gain;
          }
          base_speed = g_line_base_speed - LINE_VISION_SLOW_GAIN * fabsf(vision_err);
          if (base_speed > line_wheel_max) base_speed = line_wheel_max;
          if (base_speed < line_search_floor) base_speed = line_search_floor;

          g_pid_out = PID_realize(&pid_pidHW_Tracking, track_state, dt);
          {
            float turn_cap = LINE_VISION_TURN_CAP_ABS + 0.25f * err_mag;
            float turn_cap_ratio = base_speed * (LINE_VISION_TURN_CAP_RATIO + 0.25f * err_mag);
            float turn_cap_by_min = base_speed - track_output_min;
            if (g_line_corner_active)
            {
              turn_cap += LINE_VISION_CORNER_EXTRA_TURN_CAP;
              turn_cap_ratio = base_speed * (LINE_VISION_TURN_CAP_RATIO + LINE_VISION_CORNER_EXTRA_TURN_RATIO + 0.25f * err_mag);
            }
            if (turn_cap_ratio < turn_cap) turn_cap = turn_cap_ratio;
            if (turn_cap_by_min < turn_cap) turn_cap = turn_cap_by_min;
            if (turn_cap < 0.0f) turn_cap = 0.0f;
            g_pid_out = clampf(g_pid_out, -turn_cap, turn_cap);
            {
              float delta_turn = g_pid_out - g_line_turn_cmd_f;
              float turn_slew_step = LINE_VISION_TURN_SLEW_STEP + 0.06f * fabsf(vision_err);
              if (g_line_corner_active) turn_slew_step += LINE_VISION_CORNER_EXTRA_SLEW;
              if (turn_slew_step > 0.10f) turn_slew_step = 0.10f;
              if (delta_turn > turn_slew_step) delta_turn = turn_slew_step;
              if (delta_turn < -turn_slew_step) delta_turn = -turn_slew_step;
              g_line_turn_cmd_f += delta_turn;
              /*
               * 斜坡限速后 turn_cmd 可能仍保留上一周期的大转向量（例如刚出弯）。
               * 这里再次按当前周期 turn_cap 限幅，并同步夹紧内部状态，避免绕过转向上限。
               */
              g_line_turn_cmd_f = clampf(g_line_turn_cmd_f, -turn_cap, turn_cap);
              g_pid_out = g_line_turn_cmd_f;
            }
          }
          base_speed -= LINE_VISION_TURN_SLOW_GAIN * fabsf(g_pid_out);
          if (base_speed > line_wheel_max) base_speed = line_wheel_max;
          if (base_speed < line_search_floor) base_speed = line_search_floor;
          g_pid_out1 = base_speed + g_pid_out;
          g_pid_out2 = base_speed - g_pid_out;
          g_pid_out1 = clampf(g_pid_out1, track_output_min, line_wheel_max);
          g_pid_out2 = clampf(g_pid_out2, track_output_min, line_wheel_max);
          g_line_last_track_speed = base_speed;
          g_line_last_track_turn = g_pid_out;
          motorPidSetSpeed(g_pid_out1, g_pid_out2);
        }
        else if (g_line_vision_state == LINE_VISION_SEARCH)
        {
          /* 丢线搜索时沿上次偏差方向原地偏转，避免左右来回抖动。 */
          TickType_t search_elapsed = now_tick - g_line_search_enter_tick;
          uint8_t keep_turn_speed = 0U;
          float search_speed = fminf(g_line_search_speed, LINE_VISION_SEARCH_SPEED_CAP);
          float line_wheel_max = fminf(g_line_max_speed, LINE_VISION_TRACK_WHEEL_MAX_CAP);
          float search_output_min = g_line_corner_active ? LINE_VISION_CORNER_INNER_MIN : g_line_min_speed;
          int8_t search_sign = g_line_search_latched_sign;
          float turn = (g_line_corner_active ? LINE_VISION_SEARCH_CORNER_TURN : LINE_VISION_SEARCH_TURN) * (float)search_sign;
          if ((search_elapsed <= pdMS_TO_TICKS(LINE_VISION_SEARCH_KEEP_SPEED_MS)) &&
              (g_line_corner_active || (fabsf(g_line_last_track_turn) >= LINE_VISION_SEARCH_KEEP_TURN_TH)))
          {
            keep_turn_speed = 1U;
          }
          if (search_output_min < 0.0f) search_output_min = 0.0f;
          if (line_wheel_max < g_line_min_speed) line_wheel_max = g_line_min_speed;
          if (keep_turn_speed)
          {
            if (g_line_last_track_speed > search_speed) search_speed = g_line_last_track_speed;
            {
              float keep_turn = fabsf(g_line_last_track_turn);
              if (keep_turn > fabsf(turn))
              {
                turn = keep_turn * (float)search_sign;
              }
            }
          }
          if (search_speed < search_output_min) search_speed = search_output_min;
          if (search_speed > line_wheel_max) search_speed = line_wheel_max;
          float turn_cap = search_speed - search_output_min;
          if (turn_cap < 0.0f) turn_cap = 0.0f;
          turn = clampf(turn, -turn_cap, turn_cap);
          {
            float delta_turn = turn - g_line_turn_cmd_f;
            float search_slew = LINE_VISION_TURN_SLEW_STEP + (g_line_corner_active ? LINE_VISION_CORNER_EXTRA_SLEW : 0.0f);
            if (delta_turn > search_slew) delta_turn = search_slew;
            if (delta_turn < -search_slew) delta_turn = -search_slew;
            g_line_turn_cmd_f += delta_turn;
            /*
             * 与 TRACK 同理：SEARCH 在斜坡后也要再按 turn_cap 限幅，
             * 否则从大转向状态切入 SEARCH 时会短时超过当前搜索转向上限。
             */
            g_line_turn_cmd_f = clampf(g_line_turn_cmd_f, -turn_cap, turn_cap);
            turn = g_line_turn_cmd_f;
          }
          g_pid_out1 = search_speed + turn;
          g_pid_out2 = search_speed - turn;
          g_pid_out1 = clampf(g_pid_out1, search_output_min, line_wheel_max);
          g_pid_out2 = clampf(g_pid_out2, search_output_min, line_wheel_max);
          motorPidSetSpeed(g_pid_out1, g_pid_out2);
        }
        else
        {
          /* 长时间没有可靠视觉数据，直接停车，避免盲跑。 */
          g_pid_out = 0.0f;
          g_pid_out1 = 0.0f;
          g_pid_out2 = 0.0f;
          g_line_turn_cmd_f = 0.0f;
          g_line_corner_active = 0U;
          g_line_last_track_speed = 0.0f;
          g_line_last_track_turn = 0.0f;
          g_line_search_enter_tick = 0U;
          motorPidSetSpeed(0,0);
        }
        break;
      }

      // 避障（定时状态机，避免在一个周期里完成整套动作）
      case 2:

        TickType_t  now = xTaskGetTickCount();
        const float avoid_enter_cm = 20.0f;
        const float avoid_exit_cm = 28.0f;

        /* 距离足够时正常前进；过近时进入定时避障状态机。 */
        if ((avoid_state == AVOID_IDLE) && (snap.dist > avoid_enter_cm))
        {
          avoid_state = AVOID_IDLE;
          motorPidSetSpeed(2, 2);
          break;
        }
        switch (avoid_state)
        {
          case AVOID_IDLE:
            /* 先短暂停车，让车体稳定再开始后退。 */
            motorPidSetSpeed(0, 0);
            avoid_set_state(AVOID_BACK, 100); // 停车 100ms
            break;

          case AVOID_BACK:
            if (now >= avoid_deadline) {
              /* 定时后退，拉开与障碍物距离。 */
              motorPidSetSpeed(-1.5f, -1.5f);
              avoid_set_state(AVOID_TURN, 300); // 后退 300ms
            }
            break;

          case AVOID_TURN:
            if (now >= avoid_deadline) {
              /* 差速转向绕开障碍物。 */
              motorPidSetSpeed(2, -2);
              avoid_set_state(AVOID_PAUSE, 400); // 右转 400ms
            }
            break;

          case AVOID_PAUSE:
            if (now >= avoid_deadline) {
              /* 结束动作后短暂停顿，再回到 IDLE。 */
              if (snap.dist > avoid_exit_cm) {
                motorPidSetSpeed(0, 0);
                avoid_set_state(AVOID_IDLE, 200);
              } else {
                motorPidSetSpeed(2, -2);
                avoid_set_state(AVOID_PAUSE, 200);
              }
            }
            break;
        }
        break;

      // 跟随（距离 PID 输出前后速度）
      case 3:
        /* 跟随模式：用超声距离 PID 输出前进/后退速度。 */
        if ((snap.sr04 > 0.0f) && (snap.sr04 < 60.0f))
        {
          g_follow_pid_out = PID_realize(&pidFollow, snap.sr04, dt);


          /* 再做一道输出限幅，避免目标速度过大。 */
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

      // 走直线（IMU 航向保持）
      case 4:
      {
        /*
         * 航向保持直行：
         * 目标 yaw 与当前 yaw 做差，PID 只负责左右轮小幅差速修正。
         */
        float speed_sync;
        float yaw_err = mpu6050Movement.target_val - snap.yaw;
        float yaw_equiv;
        float base_speed = 2.0f;
        float yaw_out_cap = 0.8f;
        /* 角度差归一化到 [-180,180]，避免跨 360/0 度时误差跳变。 */
        while (yaw_err > 180.0f) yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;
        yaw_equiv = mpu6050Movement.target_val - yaw_err;
        if (fabsf(yaw_err) > 6.0f) {
          base_speed = 1.6f;
          yaw_out_cap = 1.0f;
        }
        if (fabsf(yaw_err) > 18.0f) {
          base_speed = 1.1f;
          yaw_out_cap = 1.2f;
        }
        if (fabsf(yaw_err) > 35.0f) {
          base_speed = 0.7f;
          yaw_out_cap = 1.4f;
        }

        if (fabsf(yaw_err) < 1.5f)
        {
          /* 小误差死区：减少来回微调导致的电机抖动。 */
          g_fMPU6050YawMovePidOut = 0.0f;
        }
        else
        {
          g_fMPU6050YawMovePidOut = PID_realize(&mpu6050Movement, yaw_equiv,dt);
        }

        /*
         * 仅靠当前 PID(kp 较小) 在“被外力突然踢偏”时纠偏偏弱。
         * 给中大角度误差增加一个最小转向量，保证车子确实开始回头。
         */
        if (g_fMPU6050YawMovePidOut > yaw_out_cap) g_fMPU6050YawMovePidOut = yaw_out_cap;
        if (g_fMPU6050YawMovePidOut < -yaw_out_cap) g_fMPU6050YawMovePidOut = -yaw_out_cap;

//        speed_sync = 0.15f * (Motor2Speed - Motor1Speed);
//        if (speed_sync > 0.35f) speed_sync = 0.35f;
//        if (speed_sync < -0.35f) speed_sync = -0.35f;

//        g_fMPU6050YawMovePidOut1 = 1.5f + g_fMPU6050YawMovePidOut + speed_sync;
        g_fMPU6050YawMovePidOut1 = base_speed + g_fMPU6050YawMovePidOut;

//        g_fMPU6050YawMovePidOut2 = 1.5f - g_fMPU6050YawMovePidOut - speed_sync;
        g_fMPU6050YawMovePidOut2 = base_speed - g_fMPU6050YawMovePidOut;

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
    // 逻辑任务节拍固定在 20ms，便于模式控制参数按时间尺度调节
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
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
  /*
   * 本地 OLED 参数菜单任务：
   * - 短按：上下移动/数值加减
   * - 长按：进入/确认 或 返回/取消
   * - 编辑完成后只标记 settings dirty，由后台任务延迟保存到 Flash
   */
  OLED_Init();
  OLED_Clear();
  // KEY1：上/确认，KEY2：下/返回（支持短按/长按）。
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
    // UI 循环分成三部分：采样按键 -> 更新状态机 -> 按需重绘
    // 这样逻辑清晰，也便于后续增加第三个按键/更多页面。
    //获取时间
    uint32_t now = HAL_GetTick();
    //储存长按还是短按
    KeyEvent up_evt = {0};
    KeyEvent down_evt = {0};
    uint8_t need_redraw = 0;

    //查看高低电平
    /* 先采样按键电平，再交给去抖逻辑转换成短按/长按事件。 */
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
      // 正常菜单浏览：短按移动光标，长按进入
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
        // 翻页：确保光标始终落在可视窗口内
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
              /* 进入编辑前先备份旧值，便于长按返回时撤销修改。 */
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
    // 编辑参数页：短按加减，长按上确认/下取消
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
          /* 这里只打脏，不在 UI 任务里直接写 Flash，避免按键卡顿。 */
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
// 万能数据读写助手，UI 逻辑都统一按 32 位处理
static float clampf(float v, float vmin, float vmax)
{
  /* 通用限幅工具：模式控制、UI 参数编辑都会用到。 */
  if (v < vmin) return vmin;
  if (v > vmax) return vmax;
  return v;
}

static int32_t ui_get_int(const MenuItem *item)
{
  /* 把不同整数类型统一读成 int32，UI 逻辑层就不用分支处理类型。 */
  if (item == 0 || item->value_ptr == 0) return 0;
  if (item->value_type == MENU_VALUE_INT8) return *(int8_t *)item->value_ptr;
  if (item->value_type == MENU_VALUE_INT16) return *(int16_t *)item->value_ptr;
  if (item->value_type == MENU_VALUE_INT32) return *(int32_t *)item->value_ptr;
  return 0;
}
static void ui_set_int(const MenuItem *item, int32_t v)
{
  /* 与 ui_get_int 配套：UI 层统一操作 int32，再写回实际类型。 */
  if (item == 0 || item->value_ptr == 0) return;
  if (item->value_type == MENU_VALUE_INT8) *(int8_t *)item->value_ptr = (int8_t)v;
  else if (item->value_type == MENU_VALUE_INT16) *(int16_t *)item->value_ptr = (int16_t)v;
  else if (item->value_type == MENU_VALUE_INT32) *(int32_t *)item->value_ptr = (int32_t)v;
}

//将历史的放在k里，更新evt
static void ui_key_update(KeyState *k, uint8_t raw, uint32_t now, KeyEvent *evt)
{
  /*
   * 按键状态机：
   * - 先做去抖（电平稳定一段时间才确认）
   * - 松手且未触发长按 -> short_press
   * - 按住超过阈值 -> long_press（只触发一次）
   */
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

static void ui_format_float_fixed(float v, uint8_t decimals, char *buf, uint8_t len)
{
  /*
   * 固定小数格式化的目标不是“最通用”，而是“在 MCU 上足够稳”：
   * - 限制小数位 <= 3，避免格式化过重；
   * - 处理 nan/溢出文本，防止 OLED 显示异常字符串；
   * - 手动四舍五入，减少不同 libc 对 printf 浮点支持差异。
   */
  uint32_t scale = 1U;
  uint8_t i;
  uint8_t neg;
  float abs_v;
  uint32_t scaled;
  uint32_t ipart;
  uint32_t fpart;

  if (buf == NULL || len == 0U) return;

  if (!isfinite(v))
  {
    snprintf(buf, len, "nan");
    return;
  }

  if (decimals > 3U) decimals = 3U;
  for (i = 0; i < decimals; i++) scale *= 10U;

  neg = (v < 0.0f) ? 1U : 0U;
  abs_v = neg ? (-v) : v;

  if (abs_v > 99999.0f)
  {
    snprintf(buf, len, neg ? "-ovf" : "ovf");
    return;
  }

  scaled = (uint32_t)(abs_v * (float)scale + 0.5f);
  ipart = (scale > 0U) ? (scaled / scale) : scaled;
  fpart = (scale > 0U) ? (scaled % scale) : 0U;

  if (decimals == 0U)
  {
    snprintf(buf, len, "%s%lu", neg ? "-" : "", (unsigned long)ipart);
  }
  else
  {
    snprintf(buf, len, "%s%lu.%0*lu",
             neg ? "-" : "",
             (unsigned long)ipart,
             (int)decimals,
             (unsigned long)fpart);
  }
}

static void ui_format_value(const MenuItem *item, char *buf, uint8_t len)
{
  /* 菜单显示层统一把值格式化成字符串，避免绘图函数关心数据类型。 */
  if (item->value_type == MENU_VALUE_FLOAT && item->value_ptr)
  {
    ui_format_float_fixed(*(float *)item->value_ptr, 2U, buf, len);
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
  /* 菜单是平铺数组 + parent 索引关系，这里按 parent 动态计算子项数。 */
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
  /* OLED 每行固定宽度，先补空格再输出，防止短字符串残留旧字符。 */
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
  /* 菜单绘制同时兼容浏览态和编辑态：'>' 当前光标，'*' 当前编辑项。 */
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
  /* 状态页只展示关键运行量，刷新频率比菜单页更高。 */
  char line[32];
  char lbuf[12];
  char rbuf[12];
  char dbuf[12];
  char mbuf[12];
  char ybuf[12];

  ui_format_float_fixed(Motor1Speed, 1U, lbuf, sizeof(lbuf));
  ui_format_float_fixed(Motor2Speed, 1U, rbuf, sizeof(rbuf));
  ui_format_float_fixed(dist, 1U, dbuf, sizeof(dbuf));
  ui_format_float_fixed(mile, 1U, mbuf, sizeof(mbuf));
  ui_format_float_fixed(yaw, 1U, ybuf, sizeof(ybuf));
  snprintf(line, sizeof(line), "Mode:%d", g_ucMode);
  ui_draw_line(0, line);
  snprintf(line, sizeof(line), "L:%s R:%s", lbuf, rbuf);
  ui_draw_line(1, line);
  snprintf(line, sizeof(line), "D:%s M:%s", dbuf, mbuf);
  ui_draw_line(2, line);
  snprintf(line, sizeof(line), "Yaw:%s", ybuf);
  ui_draw_line(3, line);
}



void vApplicationMallocFailedHook(void)
{
  /* 动态分配失败（通常是某处误用 pvPortMalloc 或栈设置过大）。 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  taskDISABLE_INTERRUPTS();
  for(;;)
  {
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  /* 任务栈溢出：点亮/拉低 LED 后停机，方便现场定位。 */
  (void)xTask;
  (void)pcTaskName;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  taskDISABLE_INTERRUPTS();
  for(;;)
  {
  }
}

/* USER CODE END Application */















