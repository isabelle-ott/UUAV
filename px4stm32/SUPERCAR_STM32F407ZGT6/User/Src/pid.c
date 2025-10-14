#include "pid.h"

#define Integral_MAX 100
#define ouput_limit 10.8

extern Encoder encoder_;

void PID_Init(PID_Struct *PID_Struct, float p_, float i_, float d_)
{
    PID_Struct->kp = p_;
    PID_Struct->ki = i_;
    PID_Struct->kd = d_;
    PID_Struct->input = 0;
    PID_Struct->target = 0;
    PID_Struct->last_error = 0;
    PID_Struct->error = 0;
    PID_Struct->output = 0;
    PID_Struct->output_limit = ouput_limit;
    PID_Struct->integral = 0;
    PID_Struct->derivative = 0;
}

float PID_Calculate(PID_Struct *PID_Struct)
{
    static float dt = 0;
    static uint32_t last_t = 0;
    dt = (float)(HAL_GetTick() - last_t) / (float)1000; // 单位:秒
    last_t = HAL_GetTick();
    PID_Struct->last_error = PID_Struct->error; // 记录上一次误差
    PID_Struct->error = PID_Struct->target - PID_Struct->input;
    if (dt != 0)
    {
        PID_Struct->derivative = (PID_Struct->error - PID_Struct->last_error) / dt;
    }
    PID_Struct->integral += (PID_Struct->error * dt);
    if (PID_Struct->integral >= Integral_MAX)
    {
        PID_Struct->integral = Integral_MAX;
    }
    else if (PID_Struct->integral <= -Integral_MAX)
    {
        PID_Struct->integral = -Integral_MAX;
    }
    PID_Struct->output = (PID_Struct->kp * PID_Struct->error) + (PID_Struct->ki * PID_Struct->integral) + (PID_Struct->kd * PID_Struct->derivative);
    if (PID_Struct->output >= PID_Struct->output_limit)
    {
        PID_Struct->output = PID_Struct->output_limit;
    }
    else if (PID_Struct->output <= -(PID_Struct->output_limit))
    {
        PID_Struct->output = -(PID_Struct->output_limit);
    }
    return PID_Struct->output;
}

void PID_set_target(PID_Struct *PID_Struct, float target)
{
    PID_Struct->target = target; // 设定目标
}

// 读取输入
void PID_motor1_Getinput(PID_Struct *PID_Struct)
{
    PID_Struct->input = Encoder_GetLeftFrontVel(&encoder_);
}

void PID_motor2_Getinput(PID_Struct *PID_Struct)
{
    PID_Struct->input = Encoder_GetLeftRearVel(&encoder_);
}

void PID_motor3_Getinput(PID_Struct *PID_Struct)
{
    PID_Struct->input = Encoder_GetRightRearVel(&encoder_);
}

void PID_motor4_Getinput(PID_Struct *PID_Struct)
{
    PID_Struct->input = Encoder_GetRightFrontVel(&encoder_);
}
