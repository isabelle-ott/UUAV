#include "motor.h"

extern PID_Struct motor1_PID;
extern PID_Struct motor2_PID;
extern PID_Struct motor3_PID;
extern PID_Struct motor4_PID;

void Motor_StartPWM()
{
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

void motor1_SetVelocity(float velocity)
{
    velocity = -velocity;
    if (velocity > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }

    float speed_abs = fabsf(velocity);
    float pwm_value_1 = 0;
    if (velocity >= 0)
    {
        pwm_value_1 = (speed_abs / 10.8 * 2100);
    }
    else
    {
        pwm_value_1 = ((10.8 - speed_abs) / 10.8 * 2100);
    }
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, pwm_value_1);
}

void motor2_SetVelocity(float velocity)
{
    velocity = -velocity;

    if (velocity > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    }

    float speed_abs = fabsf(velocity);
    float pwm_value = 0;
    if (velocity >= 0)
    {
        pwm_value = (speed_abs / 10.8 * 2100);
    }
    else
    {
        pwm_value = ((10.8 - speed_abs) / 10.8 * 2100);
    }

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, pwm_value);
}

void motor3_SetVelocity(float velocity)
{
    if (velocity > 0)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
    }

    float speed_abs = fabsf(velocity);
    float pwm_value = 0;

    if (velocity >= 0)
    {
        pwm_value = (speed_abs / 10.8 * 2100);
    }
    else
    {
        pwm_value = ((10.8 - speed_abs) / 10.8 * 2100);
    }

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwm_value);
}

void motor4_SetVelocity(float velocity)
{
    if (velocity > 0)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    }

    float speed_abs = fabsf(velocity);
    float pwm_value = 0;

    if (velocity >= 0)
    {
        pwm_value = (speed_abs / 10.8 * 2100);
    }
    else
    {
        pwm_value = ((10.8 - speed_abs) / 10.8 * 2100);
    }

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, pwm_value);
}

void motor1_control(float target)
{
    PID_set_target(&motor1_PID, target);
    PID_motor1_Getinput(&motor1_PID);
    motor1_SetVelocity(PID_Calculate(&motor1_PID));
}

void motor2_control(float target)
{
    PID_set_target(&motor2_PID, target);
    PID_motor2_Getinput(&motor2_PID);
    motor2_SetVelocity(PID_Calculate(&motor2_PID));
}

void motor3_control(float target)
{
    PID_set_target(&motor3_PID, target);
    PID_motor3_Getinput(&motor3_PID);
    motor3_SetVelocity(PID_Calculate(&motor3_PID));
}

void motor4_control(float target)
{
    PID_set_target(&motor4_PID, target);
    PID_motor4_Getinput(&motor4_PID);
    motor4_SetVelocity(PID_Calculate(&motor4_PID));
}