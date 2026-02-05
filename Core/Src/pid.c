//
// Created by miyeon on 2025/11/28.
//

#include "pid.h"

tpid pidMotor1Speed;
tpid pidMotor2Speed;
tpid pid_pidHW_Tracking;
tpid pidFollow;
tpid mpu6050Movement ;



void PID_init(void)
{
    pidMotor1Speed.actual_val=0.0;
    pidMotor1Speed.target_val=0.0;
    pidMotor1Speed.output_val=0.0;
    pidMotor1Speed.err=0.0;
    pidMotor1Speed.err_last=0.0;
    pidMotor1Speed.err_sum=0.0;
    pidMotor1Speed.kd=0.0;
    pidMotor1Speed.ki=0.1;
    pidMotor1Speed.kp=8.0;
    pidMotor1Speed.max_output=100.0;
    pidMotor1Speed.max_iout=500.0;
    pidMotor1Speed.d_filter_alpha=0.2;
    pidMotor1Speed.d_out=0.0;

    pidMotor2Speed.actual_val=0.0;
    pidMotor2Speed.target_val=0.0;
    pidMotor2Speed.output_val=0.0;
    pidMotor2Speed.err=0.0;
    pidMotor2Speed.err_last=0.0;
    pidMotor2Speed.err_sum=0.0;
    pidMotor2Speed.kd=0.0;
    pidMotor2Speed.ki=0.1;
    pidMotor2Speed.kp=8.0;
    pidMotor2Speed.max_output=100.0;
    pidMotor2Speed.max_iout=500.0;
    pidMotor2Speed.d_filter_alpha=0.2;
    pidMotor2Speed.d_out=0.0;

    pid_pidHW_Tracking.actual_val=0.0;
    pid_pidHW_Tracking.target_val=0.0;
    pid_pidHW_Tracking.output_val=0.0;
    pid_pidHW_Tracking.err=0.0;
    pid_pidHW_Tracking.err_last=0.0;
    pid_pidHW_Tracking.err_sum=0.0;
    pid_pidHW_Tracking.kd=1;
    pid_pidHW_Tracking.ki=0.0;
    pid_pidHW_Tracking.kp=-2;
    pid_pidHW_Tracking.max_output=100.0;
    pid_pidHW_Tracking.max_iout=500.0;
    pid_pidHW_Tracking.d_filter_alpha=0.2;
    pid_pidHW_Tracking.d_out=0.0;


    pidFollow.actual_val=0.0;
    pidFollow.target_val=22.50;
    pidFollow.output_val=0.0;
    pidFollow.err=0.0;
    pidFollow.err_last=0.0;
    pidFollow.err_sum=0.0;
    pidFollow.kp=-0.5;
    pidFollow.ki=-0.001;
    pidFollow.kd=0;
    pidFollow.max_output=100.0;
    pidFollow.max_iout=500.0;
    pidFollow.d_filter_alpha=0.2;
    pidFollow.d_out=0.0;

    mpu6050Movement.actual_val=0.0;
    mpu6050Movement.target_val=0.0;
    mpu6050Movement.output_val=0.0;
    mpu6050Movement.err=0.0;
    mpu6050Movement.err_last=0.0;
    mpu6050Movement.err_sum=0.0;
    mpu6050Movement.kd=0.1;
    mpu6050Movement.ki=0.0;
    mpu6050Movement.kp=0.02;
    mpu6050Movement.max_output=100.0;
    mpu6050Movement.max_iout=500.0;
    mpu6050Movement.d_filter_alpha=0.2;
    mpu6050Movement.d_out=0.0;


}


float P_realize(tpid * pid,float actual_val)
{
    pid->actual_val=actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->actual_val = pid->kp * pid->err ;

    return pid->actual_val;
}



float PI_realize(tpid * pid,float actual_val)
{
    pid->actual_val=actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->err_sum += pid->err;
    pid->actual_val = pid->kp * pid->err + pid->ki * pid->err_sum;
    return pid->actual_val;
}

float PID_realize(tpid * pid,float actual_val,float dt)
{
    float d_raw;
    float i_candidate;
    float output_raw;
    float output_limited;
    int saturating;

    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;

    i_candidate = pid->err_sum + pid->err * dt;
    if (pid->max_iout > 0.0)
    {
        if (i_candidate > pid->max_iout) i_candidate = pid->max_iout;
        if (i_candidate < -pid->max_iout) i_candidate = -pid->max_iout;
    }

    //修正d的 用滤波
    d_raw = (pid->err - pid->err_last)/ dt ;
    pid->d_out += pid->d_filter_alpha * (d_raw - pid->d_out);

    output_raw = pid->kp * pid->err + pid->ki * i_candidate + pid->kd * pid->d_out;
    output_limited = output_raw;
    if (pid->max_output > 0.0)
    {
        if (output_limited > pid->max_output) output_limited = pid->max_output;
        if (output_limited < -pid->max_output) output_limited = -pid->max_output;
    }

    saturating = (pid->max_output > 0.0) && (output_limited != output_raw);
    if (!(saturating && ((output_raw > pid->max_output && pid->err > 0.0) ||
        (output_raw < -pid->max_output && pid->err < 0.0))))
    {
        pid->err_sum = i_candidate;
    }

    pid->output_val = output_limited;
    pid->err_last = pid->err;

    return pid->output_val;

}

void PID_Reset(tpid * pid)
{
    pid->actual_val = 0.0;
    pid->err = 0.0;
    pid->err_last = 0.0;
    pid->err_sum = 0.0;
    pid->d_out = 0.0;
    pid->output_val = 0.0;
}

