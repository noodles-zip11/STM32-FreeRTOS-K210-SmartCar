//
// Created by miyeon on 2025/11/28.
//

#ifndef CAR_PID_H
#define CAR_PID_H

typedef struct
{
    float targer_val;
    float actual_val;
    float err ;
    float err_last;
    float err_sum;
    float kp, ki, kd;
    float output_val;
}tpid;

float P_realize(tpid * pid,float actual_val);
void PID_init(void);
float PI_realize(tpid * pid,float actual_val);
float PID_realize(tpid * pid,float actual_val);


#endif //CAR_PID_H




