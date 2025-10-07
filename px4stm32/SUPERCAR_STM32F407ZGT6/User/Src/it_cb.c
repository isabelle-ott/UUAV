#include "it_cb.h"
#include "init.h"    // 包含encoder_全局变量声明
#include "encoder.h" // 包含Encoder_Sample函数声明

// TIM6中断回调函数（采样周期由TIM6的ARR值决定，如10ms采样一次）
// 在定时器中断回调中
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        // 采样编码器数据
        Encoder_Sample(&encoder_);

        // 从编码器更新里程计数据
        EncoderOdom_UpdateFromEncoder(&encoder_odom_, &encoder_);

        // 计算里程计
        EncoderOdom_Calculate(&encoder_odom_);

        MotorControl_Loop(&motor_control_);

        PositionControl_Loop(&position_control_); // 位置控制闭环（同频率）
    }
}
