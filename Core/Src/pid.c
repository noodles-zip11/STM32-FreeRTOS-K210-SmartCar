//
// Created by miyeon on 2025/11/28.
//

#include "pid.h"

tpid pidMotor1Speed;
tpid pidMotor2Speed;
tpid pid_pidHW_Tracking;
tpid pidFollow;
tpid mpu6050Movement ;

/*
 * 本文件提供两层能力：
 * 1) 各业务 PID 实例的参数/状态初始化（PID_init）
 * 2) 通用 PID 计算与状态清零（PID_realize / PID_Reset）
 *
 * 约定：
 * - target_val 在外部任务中设置
 * - actual_val 由 realize 函数每次调用时传入
 * - output_val 为本次控制输出（由调用方决定如何使用）
 */


/*
 * 统一初始化本项目里用到的几个 PID 实例。
 * 这样做的目的不是“图省事”，而是避免某个控制器忘记清积分/误差缓存，
 * 上电后直接带着脏状态运行，导致一开始就抖动或冲一下。
 */
void PID_init(void)
{
    /*
     * 这里只初始化“默认参数 + 运行状态清零”。
     * 如果工程启用了参数持久化，后续会由 settings_load() 覆盖默认参数。
     */
    /* 左轮速度环：输出最终会映射到 PWM，占空比上限按 100 附近设置。 */
    pidMotor1Speed.actual_val=0.0;
    pidMotor1Speed.target_val=0.0;
    pidMotor1Speed.output_val=0.0;
    pidMotor1Speed.err=0.0;
    pidMotor1Speed.err_last=0.0;
    pidMotor1Speed.err_sum=0.0;
    pidMotor1Speed.kd=0.0;
    pidMotor1Speed.ki=1.5;
    pidMotor1Speed.kp=16.0;
    pidMotor1Speed.max_output=100.0;
    pidMotor1Speed.max_iout=500.0;
    pidMotor1Speed.d_filter_alpha=0.2;
    pidMotor1Speed.d_out=0.0;

    /* 右轮速度环：先和左轮保持一致，后期可单独微调补偿机械差异。 */
    pidMotor2Speed.actual_val=0.0;
    pidMotor2Speed.target_val=0.0;
    pidMotor2Speed.output_val=0.0;
    pidMotor2Speed.err=0.0;
    pidMotor2Speed.err_last=0.0;
    pidMotor2Speed.err_sum=0.0;
    pidMotor2Speed.kd=0.0;
    pidMotor2Speed.ki=1.5;
    pidMotor2Speed.kp=16.0;
    pidMotor2Speed.max_output=100.0;
    pidMotor2Speed.max_iout=500.0;
    pidMotor2Speed.d_filter_alpha=0.2;
    pidMotor2Speed.d_out=0.0;

    /* 巡线转向 PID：目标是偏差回零，kp 为负用于匹配当前左右轮修正方向。 */
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


    /* 跟随模式距离 PID：目标距离约 22.5cm，输出直接作为前进/后退速度。 */
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

    /* 航向保持微调 PID：只负责纠偏，不负责主速度，所以 kp 设置较小。 */
    mpu6050Movement.actual_val=0.0;
    mpu6050Movement.target_val=0.0;
    mpu6050Movement.output_val=0.0;
    mpu6050Movement.err=0.0;
    mpu6050Movement.err_last=0.0;
    mpu6050Movement.err_sum=0.0;
    mpu6050Movement.kd=0.0;
    mpu6050Movement.ki=0.0;
    mpu6050Movement.kp=0.02;
    mpu6050Movement.max_output=100.0;
    mpu6050Movement.max_iout=500.0;
    mpu6050Movement.d_filter_alpha=0.2;
    mpu6050Movement.d_out=0.0;


}


float P_realize(tpid * pid,float actual_val)
{
    /*
     * 纯 P 控制：结构简单、响应快，但一般会留稳态误差。
     * 这里沿用历史写法：函数返回输出值，同时复用 pid->actual_val 字段暂存输出。
     * （实际测量值在计算后不再保留在 actual_val 中）
     */
    pid->actual_val=actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->actual_val = pid->kp * pid->err ;

    return pid->actual_val;
}



float PI_realize(tpid * pid,float actual_val)
{
    /*
     * 在 P 的基础上加积分，解决“误差一直差一点点”的情况。
     * 同样沿用历史写法：返回值和 pid->actual_val 都表示控制输出。
     */
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

    /*
     * 这里是项目里最常用的速度/方向控制器：
     * - 用 dt 计算积分和微分，适配任务周期微小抖动；
     * - 积分限幅防止累积过头；
     * - D 项低通减少编码器噪声放大；
     * - 输出限幅后做条件积分，减轻积分饱和。
     *
     * 前提条件：dt 必须 > 0。调用方（ControlTask/LogicTask）已做兜底。
     */
    // 1) 计算当前误差
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;

    // 2) 先计算“候选积分项”，再做积分限幅（不立即写回，后面还要判断是否抗饱和）
    i_candidate = pid->err_sum + pid->err * dt;
    if (pid->max_iout > 0.0)
    {
        if (i_candidate > pid->max_iout) i_candidate = pid->max_iout;
        if (i_candidate < -pid->max_iout) i_candidate = -pid->max_iout;
    }

    // 3) D 项：误差变化率 + 一阶低通滤波（抑制噪声）
    /* D 项用“误差变化率”，再做一阶低通，避免瞬时尖峰把输出拉爆。 */
    d_raw = (pid->err - pid->err_last)/ dt ;
    pid->d_out += pid->d_filter_alpha * (d_raw - pid->d_out);

    // 4) 组合 P/I/D 输出，并做总输出限幅
    output_raw = pid->kp * pid->err + pid->ki * i_candidate + pid->kd * pid->d_out;
    output_limited = output_raw;
    if (pid->max_output > 0.0)
    {
        if (output_limited > pid->max_output) output_limited = pid->max_output;
        if (output_limited < -pid->max_output) output_limited = -pid->max_output;
    }

    // 5) 条件积分（抗积分饱和）
    /* 输出已经打满且误差还在往同方向推时，不继续累积积分（抗积分饱和）。 */
    saturating = (pid->max_output > 0.0) && (output_limited != output_raw);
    if (!(saturating && ((output_raw > pid->max_output && pid->err > 0.0) ||
        (output_raw < -pid->max_output && pid->err < 0.0))))
    {
        pid->err_sum = i_candidate;
    }

    // 6) 更新状态并返回本次控制输出
    pid->output_val = output_limited;
    pid->err_last = pid->err;

    return pid->output_val;

}

void PID_Reset(tpid * pid)
{
    /*
     * 只清运行时状态，不改 kp/ki/kd 参数。
     * 常用于目标速度反向切换时，避免旧积分项造成反向瞬间过冲。
     * 也常用于模式切换（如从巡线切到跟随）时清掉上一模式残留状态。
     */
    pid->actual_val = 0.0;
    pid->err = 0.0;
    pid->err_last = 0.0;
    pid->err_sum = 0.0;
    pid->d_out = 0.0;
    pid->output_val = 0.0;
}

