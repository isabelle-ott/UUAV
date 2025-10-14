#include "it_cb.h"
#include "init.h"    // 包含encoder_全局变量声明
#include "encoder.h" // 包含Encoder_Sample函数声明

// TIM6中断回调函数
// 在定时器中断回调中
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        // 采样编码器数据
        Encoder_Sample(&encoder_);
    }
}

extern uint8_t rx_buffer;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        JY61P_ParseData(rx_buffer);
        // 重新启动DMA接收
        HAL_UART_Receive_DMA(&huart3, &rx_buffer, 1);
    }
}