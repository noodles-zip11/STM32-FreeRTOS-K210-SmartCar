//
// HC-SR04 超声波测距模块驱动
// 创建者：miyeon（2025/12/5）
//

#include "main.h"



void HC_SR04_Delayus(uint32_t usdelay)
{
    __IO uint32_t Delay = usdelay * (SystemCoreClock / 8U / 1000u / 1000);
    do
    {
        __NOP();
    }
    while (Delay --);
}

float HC_SR04_Read(void)
{
    uint32_t i=0 ;
    float Distance ;
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
    HC_SR04_Delayus(15);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);

    while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == GPIO_PIN_RESET)
    {
        i++;
        HC_SR04_Delayus(1);
        if (i > 100000) {
            return -1.0 ;
        }
    }

    i=0;
    while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == GPIO_PIN_SET)
    {
        i++;
        HC_SR04_Delayus(1);
        if (i > 100000) {
            return -2.0 ;
        }
    }
    Distance = (float)i * 2.0f * 0.034f / 2.0f;
    return Distance;
}
