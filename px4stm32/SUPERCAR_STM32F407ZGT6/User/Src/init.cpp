#include "init.h"
#include "encoder.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

// -------------------------- All_Init()实现：初始化编码器相关硬件 --------------------------
void All_Init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim6);
}
