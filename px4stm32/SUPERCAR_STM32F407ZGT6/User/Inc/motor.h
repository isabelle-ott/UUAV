#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

// 电机ID定义（与Motor_SetPWM函数参数对应）
typedef enum
{
    MOTOR_LEFT_FRONT = 0,  // 左前电机
    MOTOR_RIGHT_FRONT = 1, // 右前电机
    MOTOR_RIGHT_REAR = 2,  // 右后电机
    MOTOR_LEFT_REAR = 3    // 左后电机
} Motor_ID;

// 方向定义（与GPIO状态对应）
typedef enum
{
    MOTOR_FORWARD = 0, // GPIO为0时正转
    MOTOR_BACKWARD = 1 // GPIO为1时反转
} Motor_Direction;

// 初始化电机控制（配置GPIO和PWM定时器）
void Motor_Init(void);

// 设置电机PWM和方向
// 参数：
//   id: 电机ID（MOTOR_LEFT_FRONT等）
//   pwm: PWM占空比（-100.0f ~ 100.0f，正负表示方向）
void Motor_SetPWM(Motor_ID id, float pwm);

// 单独设置电机方向
// 参数：
//   id: 电机ID
//   dir: 方向（MOTOR_FORWARD或MOTOR_BACKWARD）
void Motor_SetDirection(Motor_ID id, Motor_Direction dir);

// 单独设置电机PWM占空比（仅调速，不改变方向）
// 参数：
//   id: 电机ID
//   pwm: PWM占空比（0.0f ~ 100.0f）
void Motor_SetSpeed(Motor_ID id, float speed);

// 停止所有电机
void Motor_StopAll(void);

#endif // MOTOR_H
