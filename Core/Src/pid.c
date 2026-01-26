//
// Created by miyeon on 2025/11/28.
//

#include "pid.h"

tpid pidMotor1Speed;
tpid pidMotor2Speed;
tpid pid_pidHW_Tracking;
tpid pidFollow;
tpid mpu6050Movement ;
tpid k210motion;



void PID_init(void)
{
    //输入0到10，输出0-100；
    pidMotor1Speed.actual_val=0.0;
    pidMotor1Speed.targer_val=0.0;
    pidMotor1Speed.output_val=0.0;
    pidMotor1Speed.err=0.0;
    pidMotor1Speed.err_last=0.0;
    pidMotor1Speed.err_sum=0.0;
    pidMotor1Speed.kd=0.0;
    pidMotor1Speed.ki=0.1;
    pidMotor1Speed.kp=8.0;

    pidMotor2Speed.actual_val=0.0;
    pidMotor2Speed.targer_val=0.0;
    pidMotor2Speed.output_val=0.0;
    pidMotor2Speed.err=0.0;
    pidMotor2Speed.err_last=0.0;
    pidMotor2Speed.err_sum=0.0;
    pidMotor2Speed.kd=0.0;
    pidMotor2Speed.ki=0.1;
    pidMotor2Speed.kp=8.0;

    pid_pidHW_Tracking.actual_val=0.0;
    pid_pidHW_Tracking.targer_val=0.0;
    pid_pidHW_Tracking.output_val=0.0;
    pid_pidHW_Tracking.err=0.0;
    pid_pidHW_Tracking.err_last=0.0;
    pid_pidHW_Tracking.err_sum=0.0;
    pid_pidHW_Tracking.kd=1;
    pid_pidHW_Tracking.ki=0.0;
    pid_pidHW_Tracking.kp=-2;


    pidFollow.actual_val=0.0;
    pidFollow.targer_val=22.50;//定距离跟随 目标距离22.5cm
    pidFollow.err=0.0;
    pidFollow.err_last=0.0;
    pidFollow.err_sum=0.0;
    pidFollow.kp=-0.5;//定距离跟随的Kp大小通过估算PID输入输出数据，确定大概大小，然后在调试
    pidFollow.ki=-0.001;//Ki小一些
    pidFollow.kd=0;

    mpu6050Movement.actual_val=0.0;
    mpu6050Movement.targer_val=0.0;
    mpu6050Movement.err=0.0;
    mpu6050Movement.err_last=0.0;
    mpu6050Movement.err_sum=0.0;
    mpu6050Movement.kd=0.1;
    mpu6050Movement.ki=0.0;
    mpu6050Movement.kp=0.02;


    k210motion.actual_val=0.0;
    k210motion.targer_val=160.0;
    k210motion.err=0.0;
    k210motion.err_last=0.0;
    k210motion.err_sum=0.0;
    k210motion.kd=0.1;
    k210motion.ki=0.0;
    k210motion.kp=0.02;

}


float P_realize(tpid * pid,float actual_val)
{
    pid->actual_val=actual_val;
    pid->err = pid->targer_val - pid->actual_val;
    pid->actual_val = pid->kp * pid->err ;

    return pid->actual_val;
}



float PI_realize(tpid * pid,float actual_val)
{
    pid->actual_val=actual_val;
    pid->err = pid->targer_val - pid->actual_val;
    pid->err_sum += pid->err;
    pid->actual_val = pid->kp * pid->err + pid->ki * pid->err_sum;
    return pid->actual_val;
}

float PID_realize(tpid * pid,float actual_val)
{
    pid->actual_val = actual_val;
    pid->err = pid->targer_val - pid->actual_val;
    pid->err_sum += pid->err;

    if(pid->err_sum > 500)  pid->err_sum = 500;
    if(pid->err_sum < -500) pid->err_sum = -500;


    pid->output_val = pid->kp * pid->err + pid->ki * pid->err_sum + pid->kd * (pid->err-pid->err_last);

    pid->err_last = pid->err;


    if(pid->output_val > 100) pid->output_val = 100;
    if(pid->output_val < -100) pid->output_val = -100;


    return pid->output_val;

}

