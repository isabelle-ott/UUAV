#include "init.h"
#include "tim.h"
#include "stdio.h"
#include <stdint.h>
#include <math.h>
#include "usart.h"
#include "dma.h"

// 已测试
Encoder encoder_;

UartPi uart_pi_;

JY61P_Acc g_jy61p_acc = {0};
JY61P_Gyro g_jy61p_gyro = {0};
JY61P_Angle g_jy61p_angle = {0};
JY61P_Tim g_jy61p_Tim = {0};
PID_Struct motor1_PID;
PID_Struct motor2_PID;
PID_Struct motor3_PID;
PID_Struct motor4_PID;

uint8_t rx_buffer = 0;

#define PID1_kp 5.4
#define PID1_ki 40
#define PID1_kd 50
#define PID2_kp 5.4
#define PID2_ki 40
#define PID2_kd 50

float p__ = 5.4;
float i__ = 40;
float d__ = 50;

void All_Init(void)
{

    // 编码器初始化
    Encoder_Init(&encoder_,
                 &htim3,
                 &htim4,
                 &htim2,
                 &htim1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    LightInit();

    HAL_TIM_Base_Start_IT(&htim6);

    UartPi_Init(&uart_pi_, &huart2);

    JY61P_Init(&huart3);
    HAL_UART_Receive_DMA(&huart3, &rx_buffer, 1);
    PID_Init(&motor1_PID, PID1_kp, PID1_ki, PID1_kd);
    PID_Init(&motor2_PID, PID1_kp, PID1_ki, PID1_kd);
    PID_Init(&motor3_PID, PID1_kp, PID1_ki, PID1_kd);
    PID_Init(&motor4_PID, PID1_kp, PID1_ki, PID1_kd);
}
