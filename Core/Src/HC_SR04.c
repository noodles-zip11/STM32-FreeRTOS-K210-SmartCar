//
// HC-SR04 超声波测距模块驱动
// 创建者：miyeon（2025/12/5）
//

#include "main.h"

/*
 * HC-SR04 超声波驱动（阻塞式实现）
 * - TRIG: PB5（输出）
 * - ECHO: PA6（输入）
 * - 返回值单位：厘米(cm)
 * - 返回负值表示超时/异常（由上层过滤）
 *
 * 当前实现使用软件延时 + 轮询，优点是简单；缺点是会占用 CPU。
 * 若后续要提高采样频率或精度，建议改成定时器输入捕获方案。
 */



void HC_SR04_Delayus(uint32_t usdelay)
{
    /*
     * 粗略微秒延时（busy-wait）：
     * 依赖 SystemCoreClock，精度会受中断和编译优化影响。
     * 对 HC-SR04 触发与超时轮询通常够用。
     */
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
    /*
     * 测距流程：
     * 1) 给 TRIG 一个 >10us 脉冲
     * 2) 等待 ECHO 拉高（超时返回 -1）
     * 3) 统计 ECHO 高电平持续时间（超时返回 -2）
     * 4) 按声速公式换算为厘米
     */
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
    HC_SR04_Delayus(15);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);

    while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == GPIO_PIN_RESET)
    {
        i++;
        HC_SR04_Delayus(1);
        if (i > 100000) {
            /* 等不到回波起始：可能超量程、无目标或接线异常。 */
            return -1.0 ;
        }
    }

    i=0;
    while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == GPIO_PIN_SET)
    {
        i++;
        HC_SR04_Delayus(1);
        if (i > 100000) {
            /* 回波持续过久：通常表示异常回波或超量程。 */
            return -2.0 ;
        }
    }
    /*
     * i 近似为 ECHO 高电平持续时间（单位约 us）。
     * 距离 = 时间 * 声速 / 2。
     * 这里保留工程当前表达式（历史标定已按该实现使用）。
     */
    Distance = (float)i * 2.0f * 0.034f / 2.0f;
    return Distance;
}
