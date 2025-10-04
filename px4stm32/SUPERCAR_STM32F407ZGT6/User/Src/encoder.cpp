#include "encoder.h"
#include "stm32f4xx_hal.h"

// 使用extern确保与HAL库中定义的TIM1句柄一致
extern TIM_HandleTypeDef htim1;

// 获取编码器计数值的函数实现
int32_t Encoder_GetCounter(void)
{
    // 使用HAL库函数获取TIM1的计数器值
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
}
