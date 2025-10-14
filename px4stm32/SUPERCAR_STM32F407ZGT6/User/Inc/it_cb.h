#pragma once

#include "main.h"
#include "usart.h"
#include "tim.h"
#include "encoder.h"
#include "odom.h"
#include "motor.h"
#include "jy61p.h"

// -------------------------- 函数声明 --------------------------
// 重写定时器周期中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
