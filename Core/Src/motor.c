//
// Created by miyeon on 2025/11/21.
//

#include "main.h"
#include "motor.h"
#include "tim.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "queue.h"


float g_TargetSpeed = 0;
extern float Motor1Speed ;
extern float Motor2Speed ;
extern  tpid pidMotor1Speed;
extern  tpid pidMotor2Speed;
extern QueueHandle_t MotorTargetQueueHandle;
void Motor_Set (int motor1,int motor2) {
    if (motor2 < 0)
    {
        AIN1_SET;
    }
    else
    {
        AIN1_RESET;
    }
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

void motorPidSetSpeed(float Motor1SetSpeed, float Motor2SetSpeed) {
    MotorTarget_t target = { Motor1SetSpeed, Motor2SetSpeed };

    if (MotorTargetQueueHandle != NULL)
    {
        xQueueOverwrite(MotorTargetQueueHandle, &target);
    }
}



void motorSpeedUp(void)
{
    // 在当前基础上加
    if(g_TargetSpeed <= MAX_SPEED_UP)
    {
        g_TargetSpeed += 0.5f;
    }

    // 发送给电机
    motorPidSetSpeed(g_TargetSpeed, g_TargetSpeed);
}



void motorSpeedCut(void)
{
    // 在当前基础上减
    if(g_TargetSpeed >= 0.5f)
    {
        g_TargetSpeed -= 0.5f;
    }
    else
    {
        g_TargetSpeed = 0; // 减到底就�?
    }


    motorPidSetSpeed(g_TargetSpeed, g_TargetSpeed);
}






