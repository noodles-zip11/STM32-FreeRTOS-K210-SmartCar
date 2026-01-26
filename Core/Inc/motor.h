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
    float left;
    float right;
} MotorTarget_t;


void Motor_Set (int motor1,int motor2);
void motorSpeedUp(void);
void motorSpeedCut(void);
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed);

#endif //CAR_MOTOR_H
