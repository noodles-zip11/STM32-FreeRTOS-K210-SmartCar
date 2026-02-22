//
// Created by miyeon on 2025/11/28.
//

#ifndef CAR_PID_H
#define CAR_PID_H

typedef struct
{
    /* 目标值与当前测量值 */
    float target_val;
    float actual_val;
    /* 当前误差、上次误差、积分累计 */
    float err ;
    float err_last;
    float err_sum;
    /* PID 参数 */
    float kp, ki, kd;
    /* 计算输出与限幅参数 */
    float output_val;
    float max_output;
    float max_iout;
    /* D 项低通滤波参数与缓存 */
    float d_filter_alpha;
    float d_out;

}tpid;

/* 纯 P 控制计算 */
float P_realize(tpid * pid,float actual_val);
/* 初始化本项目所有 PID 实例参数和状态 */
void PID_init(void);
/* PI 控制计算 */
float PI_realize(tpid * pid,float actual_val);
/* PID 控制计算（含积分限幅、D 项滤波、输出限幅） */
float PID_realize(tpid * pid,float actual_val,float dt);
/* 只清运行状态，不改 kp/ki/kd */
void PID_Reset(tpid * pid);


#endif //CAR_PID_H




