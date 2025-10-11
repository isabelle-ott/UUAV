#pragma once

#include "encoder.h"
#include "encoder_odom.h"
#include "jy61p.h"
#include "imu_odom.h"
#include "odom.h"

#include "motor.h"
#include "pid.h"
#include "motor_control.h"
#include "position_control.h"

#include "it_cb.h"
#include "light.h"
#include "uart_pi.h"

extern Encoder encoder_;
extern Motor motor_; // 不带pid
extern EncoderOdom encoder_odom_;

extern PID_Controller lf_pid;            // 左前电机速度PID
extern PID_Controller rf_pid;            // 右前电机速度PID
extern PID_Controller rr_pid;            // 右后电机速度PID
extern PID_Controller lr_pid;            // 左后电机速度PID
extern PID_Controller base_position_pid; // 底盘位置PID

extern MotorControl motor_control_;       // 带pid
extern PositionControl position_control_; // 带pid

extern UartPi uart_pi_;

extern JY61P_Acc g_jy61p_acc;
extern JY61P_Gyro g_jy61p_gyro;
extern JY61P_Angle g_jy61p_angle;

// 3. 初始化与测试函数声明
void All_Init(void);
void encoder_test(uint32_t print_interval_ms);
void motor_test(Motor *motor, uint32_t test_step_duration_ms, float test_speed);
void odom_test(const EncoderOdom *odom, uint32_t print_interval_ms);
void pid_velocity_test(PID_Controller *pid, float target_speed, uint32_t test_time_ms);
void motor_control_test(MotorControl *mc, uint32_t test_duration_ms);
void position_control_test(PositionControl *pc, uint32_t total_test_time_ms);