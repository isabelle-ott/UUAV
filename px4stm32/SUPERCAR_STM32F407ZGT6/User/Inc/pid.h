#pragma once
#include "encoder.h"
typedef struct
{
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float input;        // 输入
    float target;       // 目标
    float error;        // 误差
    float last_error;   // 上次误差
    float output;       // 输出
    float output_limit; // 输出限幅
    float integral;     // 积分项
    float derivative;   // 微分项目
} PID_Struct;

void PID_Init(PID_Struct *PID_Struct, float p_, float i_, float d_);
void PID_set_target(PID_Struct *PID_Struct, float target);
float PID_Calculate(PID_Struct *PID_Struct);
void PID_motor1_Getinput(PID_Struct *PID_Struct);
void PID_motor2_Getinput(PID_Struct *PID_Struct);
void PID_motor3_Getinput(PID_Struct *PID_Struct);
void PID_motor4_Getinput(PID_Struct *PID_Struct);
