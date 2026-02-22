//
// Created by miyeon on 2025/11/21.
//

#ifndef CAR_MOTOR_H
#define CAR_MOTOR_H

#include "main.h"

#define MAX_SPEED_UP  10.0f
#define AIN1_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
#define AIN1_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);


#define BIN1_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
#define BIN1_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

typedef struct {
    /* 左右轮目标速度（单位与控制任务里的速度定义一致）。 */
    float left;
    float right;
} MotorTarget_t;


/* 直接输出左右电机 PWM（开环，范围建议 -99~99）。 */
void Motor_Set (int motor1,int motor2);
/* 全局目标速度按步进增加，并同步下发到左右轮。 */
void motorSpeedUp(void);
/* 全局目标速度按步进减少，并同步下发到左右轮。 */
void motorSpeedCut(void);
/* 发送左右轮目标速度到控制任务（闭环入口）。 */
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed);

#endif //CAR_MOTOR_H
