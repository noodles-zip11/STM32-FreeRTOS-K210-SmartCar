//
// Created by miyeon on 2025/11/28.
//

#ifndef CAR_PID_H
#define CAR_PID_H

typedef struct
{
    float target_val;
    float actual_val;
    float err ;
    float err_last;
    float err_sum;
    float kp, ki, kd;
    float output_val;
    float max_output;
    float max_iout;
    float d_filter_alpha;
    float d_out;
}tpid;

float P_realize(tpid * pid,float actual_val);
void PID_init(void);
float PI_realize(tpid * pid,float actual_val);
float PID_realize(tpid * pid,float actual_val,float dt);


#endif //CAR_PID_H




