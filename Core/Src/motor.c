//
// Created by miyeon on 2025/11/21.
//

#include "main.h"
#include "motor.h"
#include "tim.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "queue.h"

/*
 * motor.c 的职责分两层：
 * 1) Motor_Set()：直接操作方向脚 + PWM（开环执行层）
 * 2) motorPidSetSpeed()/motorSpeedUp()/motorSpeedCut()：给 ControlTask 下发目标速度（闭环入口）
 *
 * 这样模式逻辑无需关心 PWM 映射细节，统一发“目标速度”即可。
 */

float g_TargetSpeed = 0;
extern float Motor1Speed ;
extern float Motor2Speed ;
extern  tpid pidMotor1Speed;
extern  tpid pidMotor2Speed;
extern QueueHandle_t MotorTargetQueueHandle;

/*
 * 直接给左右电机下发“开环”PWM值（范围建议 -99~99）。
 * 这里把“方向”和“占空比”分开处理：
 * 1) 先根据正负号切方向引脚；
 * 2) 再把绝对值映射到 TIM1 的 PWM 比较值。
 *
 * 为什么负数要写成 (100 + motorX)：
 * 当前这套接法里 PWM 周期按 0~99 使用，电机反转时用方向脚翻转后，
 * 仍然希望占空比落在有效区间，所以把 -99~-1 映射成 1~99。
 */
void Motor_Set (int motor1,int motor2) {
    /*
     * 通道映射（按当前硬件接线）：
     * - motor1 -> TIM1_CH1 + BIN1 方向脚
     * - motor2 -> TIM1_CH4 + AIN1 方向脚
     *
     * 先设方向，再写 PWM 占空比，可避免方向切换瞬间占空比不匹配。
     */
    if (motor2 < 0)
    {
        AIN1_SET;
    }
    else
    {
        AIN1_RESET;
    }
    // 左轮 PWM 限幅并写入
    if (motor1 < 0)
    {
        BIN1_SET;
    }
    else
    {
        BIN1_RESET;
    }


    if (motor1 < 0)
    {
        if (motor1 < -99)
        {
            motor1 = -99;
        }
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(100+motor1));
    }
    if (motor1 >= 0)
    {
        if (motor1 > 99)
        {
            motor1 = 99;
        }
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor1);
    }

    // 右轮 PWM 限幅并写入
    if (motor2 < 0)
    {
        if (motor2 < -99)
        {
            motor2 = -99;
        }
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,(100+motor2));
    }
    if (motor2 >= 0)
    {
        if (motor2 > 99)
        {
            motor2 = 99;
        }
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,motor2);
    }
}

/*
 * 给控制任务发送“目标速度”，不是直接输出 PWM。
 * 使用 xQueueOverwrite 的原因：
 * - 队列长度只有 1，控制任务每 10ms 消费一次；
 * - 我们只关心“最新目标值”，旧目标没必要排队堆积。
 */
void motorPidSetSpeed(float Motor1SetSpeed, float Motor2SetSpeed) {
    /*
     * 注意这里发送的是“速度目标值”，不是 PWM。
     * 真正的 PID 计算在 ControlTask 中执行，这样控制节拍更稳定。
     */
    MotorTarget_t target = { Motor1SetSpeed, Motor2SetSpeed };

    if (MotorTargetQueueHandle != NULL)
    {
        /* 队列未创建前（系统早期）调用会被安全忽略，不会空指针崩溃。 */
        xQueueOverwrite(MotorTargetQueueHandle, &target);
    }
}



/*
 * 以固定步进提高全局巡航速度，再同步下发到左右轮。
 * 这里维护的是“目标速度状态” g_TargetSpeed，方便按键/串口做增减速。
 */
void motorSpeedUp(void)
{
    // 在当前基础上加（步进 0.5），作为“巡航速度”粗调接口
    if(g_TargetSpeed <= MAX_SPEED_UP)
    {
        g_TargetSpeed += 0.5f;
    }

    // 把最新巡航目标同步发给左右轮（直行）
    motorPidSetSpeed(g_TargetSpeed, g_TargetSpeed);
}



/*
 * 以固定步进降低全局巡航速度。
 * 低于一个步进时直接归零，避免出现很小的目标值导致电机嗡嗡响但不动。
 */
void motorSpeedCut(void)
{
    // 在当前基础上减（步进 0.5）
    if(g_TargetSpeed >= 0.5f)
    {
        g_TargetSpeed -= 0.5f;
    }
    else
    {
        g_TargetSpeed = 0;
    }


    // 同步更新左右轮目标速度
    motorPidSetSpeed(g_TargetSpeed, g_TargetSpeed);
}






