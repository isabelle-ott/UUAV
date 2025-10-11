/*
主循环测试代码：
    Motor_SetLeftFrontVel(&motor_, 50.0f); // 取绝对值=正转
    HAL_Delay(300);
    Motor_SetLeftFrontVel(&motor_, 30.0f); // 停止当前电机
    HAL_Delay(300);
    Motor_StopAll(&motor_);
    HAL_Delay(500);

*/

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <math.h>

// 电机ID枚举
typedef enum
{
    LEFT_FRONT,  // 左前电机
    RIGHT_FRONT, // 右前电机
    RIGHT_REAR,  // 右后电机
    LEFT_REAR,   // 左后电机
    MOTOR_MAX    // 电机数量
} MotorID;

// 电机单路GPIO与PWM配置
typedef struct
{
    GPIO_TypeDef *dir_port; // 方向控制GPIO端口
    uint16_t dir_pin;       // 方向控制GPIO引脚
    uint32_t pwm_ch;        // PWM通道
} MotorSingleConfig;

// 电机核心结构体
typedef struct
{
    TIM_HandleTypeDef *pwm_tim;                // PWM定时器句柄
    uint32_t pwm_period;                       // PWM周期（ARR值，如1999）
    uint32_t max_pwm_value;                    // 最大PWM值（通常=period，占空比100%）
    uint32_t min_pwm_value;                    // 最小PWM值（通常=0，占空比0%）
    MotorSingleConfig motor_config[MOTOR_MAX]; // 4个电机的硬件配置
} Motor;

// -------------------------- 核心接口函数声明 --------------------------
void Motor_Init(Motor *motor,
                TIM_HandleTypeDef *pwm_tim,
                uint32_t pwm_period,
                const MotorSingleConfig *lf_config,
                const MotorSingleConfig *rf_config,
                const MotorSingleConfig *rr_config,
                const MotorSingleConfig *lr_config);

void Motor_SetVelocity(Motor *motor, MotorID id, float velocity);
void Motor_StopAll(Motor *motor);
void Motor_StartPWM(Motor *motor);

// -------------------------- 便捷接口（静态内联，简化调用） --------------------------
static inline void Motor_SetLeftFrontVel(Motor *motor, float speed)
{
    if (motor != NULL)
        Motor_SetVelocity(motor, LEFT_FRONT, speed);
}

static inline void Motor_SetRightFrontVel(Motor *motor, float speed)
{
    if (motor != NULL)
        Motor_SetVelocity(motor, RIGHT_FRONT, speed);
}

static inline void Motor_SetRightRearVel(Motor *motor, float speed)
{
    if (motor != NULL)
        Motor_SetVelocity(motor, RIGHT_REAR, speed);
}

static inline void Motor_SetLeftRearVel(Motor *motor, float speed)
{
    if (motor != NULL)
        Motor_SetVelocity(motor, LEFT_REAR, speed);
}

#endif // MOTOR_H
