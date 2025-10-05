#include "encoder.h"
#include "stm32f4xx_hal.h"

// 左前轮编码器计数值获取
int32_t Encoder_LeftFront_GetCounter(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
}

// 右前轮编码器计数值获取
int32_t Encoder_RightFront_GetCounter(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

// 右后轮编码器计数值获取
int32_t Encoder_RightRear_GetCounter(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
}

// 左后轮编码器计数值获取
int32_t Encoder_LeftRear_GetCounter(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
}

// 左前轮编码器计数值重置
void Encoder_LeftFront_ResetCounter(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
}

// 右前轮编码器计数值重置
void Encoder_RightFront_ResetCounter(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

// 右后轮编码器计数值重置
void Encoder_RightRear_ResetCounter(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

// 左后轮编码器计数值重置
void Encoder_LeftRear_ResetCounter(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}
