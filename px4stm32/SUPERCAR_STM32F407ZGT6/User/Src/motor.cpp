#include "motor.h"
#include "stm32f4xx_hal.h"

// 电机方向控制GPIO定义
#define LEFT_FRONT_DIR_GPIO_Port GPIOB
#define LEFT_FRONT_DIR_Pin GPIO_PIN_0 // TIM5_CH1

#define RIGHT_FRONT_DIR_GPIO_Port GPIOD
#define RIGHT_FRONT_DIR_Pin GPIO_PIN_0 // TIM5_CH3

#define LEFT_REAR_DIR_GPIO_Port GPIOE
#define LEFT_REAR_DIR_Pin GPIO_PIN_0 // TIM5_CH2

#define RIGHT_REAR_DIR_GPIO_Port GPIOB
#define RIGHT_REAR_DIR_Pin GPIO_PIN_10 // TIM5_CH4

// PWM定时器和通道定义
#define PWM_TIM_Handle &htim5
#define LEFT_FRONT_PWM_CHANNEL TIM_CHANNEL_1
#define RIGHT_FRONT_PWM_CHANNEL TIM_CHANNEL_3
#define LEFT_REAR_PWM_CHANNEL TIM_CHANNEL_2
#define RIGHT_REAR_PWM_CHANNEL TIM_CHANNEL_4

// PWM配置参数（与CubeMX配置一致）
#define PWM_PERIOD 1999 // 假设PWM定时器ARR值为1999
#define MAX_PWM_VALUE PWM_PERIOD
#define MIN_PWM_VALUE 0

// 初始化电机控制
void Motor_Init(void)
{
    // 启动PWM定时器
    HAL_TIM_PWM_Start(PWM_TIM_Handle, LEFT_FRONT_PWM_CHANNEL);
    HAL_TIM_PWM_Start(PWM_TIM_Handle, RIGHT_FRONT_PWM_CHANNEL);
    HAL_TIM_PWM_Start(PWM_TIM_Handle, LEFT_REAR_PWM_CHANNEL);
    HAL_TIM_PWM_Start(PWM_TIM_Handle, RIGHT_REAR_PWM_CHANNEL);

    // 初始化所有电机为停止状态
    Motor_StopAll();
}

// 设置电机方向
void Motor_SetDirection(Motor_ID id, Motor_Direction dir)
{
    switch (id)
    {
    case MOTOR_LEFT_FRONT:
        HAL_GPIO_WritePin(LEFT_FRONT_DIR_GPIO_Port, LEFT_FRONT_DIR_Pin, (GPIO_PinState)dir);
        break;
    case MOTOR_RIGHT_FRONT:
        HAL_GPIO_WritePin(RIGHT_FRONT_DIR_GPIO_Port, RIGHT_FRONT_DIR_Pin, (GPIO_PinState)dir);
        break;
    case MOTOR_LEFT_REAR:
        HAL_GPIO_WritePin(LEFT_REAR_DIR_GPIO_Port, LEFT_REAR_DIR_Pin, (GPIO_PinState)dir);
        break;
    case MOTOR_RIGHT_REAR:
        HAL_GPIO_WritePin(RIGHT_REAR_DIR_GPIO_Port, RIGHT_REAR_DIR_Pin, (GPIO_PinState)dir);
        break;
    default:
        break;
    }
}

// 设置电机速度（PWM占空比）
void Motor_SetSpeed(Motor_ID id, float speed)
{
    // 限制速度范围在0-100之间
    if (speed < 0.0f)
        speed = 0.0f;
    if (speed > 100.0f)
        speed = 100.0f;

    // 计算PWM值
    uint32_t pwm_value = (uint32_t)(speed / 100.0f * MAX_PWM_VALUE);

    // 设置PWM值
    switch (id)
    {
    case MOTOR_LEFT_FRONT:
        __HAL_TIM_SET_COMPARE(PWM_TIM_Handle, LEFT_FRONT_PWM_CHANNEL, pwm_value);
        break;
    case MOTOR_RIGHT_FRONT:
        __HAL_TIM_SET_COMPARE(PWM_TIM_Handle, RIGHT_FRONT_PWM_CHANNEL, pwm_value);
        break;
    case MOTOR_LEFT_REAR:
        __HAL_TIM_SET_COMPARE(PWM_TIM_Handle, LEFT_REAR_PWM_CHANNEL, pwm_value);
        break;
    case MOTOR_RIGHT_REAR:
        __HAL_TIM_SET_COMPARE(PWM_TIM_Handle, RIGHT_REAR_PWM_CHANNEL, pwm_value);
        break;
    default:
        break;
    }
}

// 设置电机PWM（包含方向和速度）
void Motor_SetPWM(Motor_ID id, float pwm)
{
    Motor_Direction dir;
    float speed;

    // 根据pwm正负确定方向
    if (pwm >= 0)
    {
        dir = MOTOR_FORWARD; // 正转：GPIO为0
        speed = pwm;
    }
    else
    {
        dir = MOTOR_BACKWARD; // 反转：GPIO为1
        speed = -pwm;
    }

    // 设置方向
    Motor_SetDirection(id, dir);

    // 设置速度（PWM占空比）
    Motor_SetSpeed(id, speed);
}

// 停止所有电机
void Motor_StopAll(void)
{
    Motor_SetPWM(MOTOR_LEFT_FRONT, 0.0f);
    Motor_SetPWM(MOTOR_RIGHT_FRONT, 0.0f);
    Motor_SetPWM(MOTOR_RIGHT_REAR, 0.0f);
    Motor_SetPWM(MOTOR_LEFT_REAR, 0.0f);
}
