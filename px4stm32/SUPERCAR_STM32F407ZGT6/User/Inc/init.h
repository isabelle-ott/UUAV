#pragma once

#include "encoder.h"
#include "jy61p.h"
#include "odom.h"

#include "motor.h"
#include "pid.h"
#include "chassis_control.h"
#include "position_control.h"

#include "it_cb.h"
#include "light.h"
#include "uart_pi.h"

extern Encoder encoder_;

extern UartPi uart_pi_;

extern JY61P_Acc g_jy61p_acc;
extern JY61P_Gyro g_jy61p_gyro;
extern JY61P_Angle g_jy61p_angle;
extern PID_Struct motor1_PID;
// 3. 初始化与测试函数声明
void All_Init(void);
void encoder_test(uint32_t print_interval_ms);
