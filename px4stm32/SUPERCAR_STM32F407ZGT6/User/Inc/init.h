#pragma once

#include "encoder.h" // 包含Encoder结构体定义
#include "encoder_odom.h"
#include "imu.h"
#include "imu_odom.h"
#include "odom.h"
#include "motor.h" // 包含MotorController、电机接口定义
#include "pid.h"
#include "motor_control.h"
#include "it_cb.h"

// 1. 原有全局实例声明
extern Encoder encoder_;
extern Motor motor_;
extern EncoderOdom encoder_odom_;

// 2. PID实例声明（4个轮子速度PID + 1个基础位置PID）
extern PID_Controller lf_pid; // 左前电机速度PID
extern PID_Controller rf_pid; // 右前电机速度PID
extern PID_Controller rr_pid; // 右后电机速度PID
extern PID_Controller lr_pid; // 左后电机速度PID
extern PID_Controller base_position_pid;

extern MotorControl motor_control_;

// 3. 初始化与测试函数声明
void All_Init(void);
void encoder_test(uint32_t print_interval_ms);
void motor_test(Motor *motor, uint32_t test_step_duration_ms, float test_speed);
void odom_test(const EncoderOdom *odom, uint32_t print_interval_ms);
void pid_velocity_test(PID_Controller *pid, float target_speed, uint32_t test_time_ms);
void motor_control_test(MotorControl *mc, uint32_t test_duration_ms);
