#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "stdint.h"

// 电机ID枚举（与Motor模块对齐，确保一致性）
typedef enum
{
    MOTOR_LF, // 左前电机
    MOTOR_RF, // 右前电机
    MOTOR_RR, // 右后电机
    MOTOR_LR, // 左后电机
    MOTOR_NUM // 电机总数（用于数组边界检查）
} MotorControlID;

// 电机控制结构体（整合电机、PID、编码器依赖）
typedef struct
{
    // 外部依赖实例指针（初始化时绑定）
    Motor *motor;                  // 电机实例（提供PWM驱动）
    Encoder *encoder;              // 编码器实例（提供速度反馈）
    PID_Controller pid[MOTOR_NUM]; // 4个电机的速度PID控制器

    // 控制参数（可配置）
    float pid_kp;             // PID比例系数
    float pid_ki;             // PID积分系数
    float pid_kd;             // PID微分系数
    float pid_integral_limit; // PID积分限幅
    float pid_output_limit;   // PID输出限幅（对应电机PWM占空比范围：-100~100）
    uint32_t control_freq;    // 控制频率（Hz，默认100Hz）
} MotorControl;

// -------------------------- 核心接口函数声明 --------------------------
/**
 * @brief 电机控制模块初始化（绑定依赖实例+初始化PID）
 * @param mc：MotorControl实例指针（如&motor_control_）
 * @param motor：Motor实例指针（已初始化的电机驱动）
 * @param encoder：Encoder实例指针（已初始化的编码器）
 * @param pid_kp/ki/kd：PID参数
 * @param pid_integral_limit：PID积分限幅
 * @param pid_output_limit：PID输出限幅（建议±100，对应PWM占空比）
 * @param control_freq：控制频率（Hz，建议50~200）
 */
void MotorControl_Init(MotorControl *mc,
                       Motor *motor,
                       Encoder *encoder,
                       float pid_kp,
                       float pid_ki,
                       float pid_kd,
                       float pid_integral_limit,
                       float pid_output_limit,
                       uint32_t control_freq);

/**
 * @brief 设置单个电机的目标速度（闭环控制入口）
 * @param mc：MotorControl实例指针
 * @param id：电机ID（MOTOR_LF~MOTOR_LR）
 * @param target_vel：目标速度（单位：r/s，与编码器速度单位一致）
 */
void MotorControl_SetTargetVel(MotorControl *mc, MotorControlID id, float target_vel);

/**
 * @brief 电机控制主循环（需按control_freq频率调用，实现PID闭环）
 * @param mc：MotorControl实例指针
 * @note 建议在定时器中断或主循环中按固定频率调用（如100Hz=10ms周期）
 */
void MotorControl_Loop(MotorControl *mc);

/**
 * @brief 停止所有电机（紧急停止，PWM输出0）
 * @param mc：MotorControl实例指针
 */
void MotorControl_StopAll(MotorControl *mc);

/**
 * @brief 获取单个电机的当前反馈速度（编码器实时速度）
 * @param mc：MotorControl实例指针
 * @param id：电机ID（MOTOR_LF~MOTOR_LR）
 * @return 反馈速度（单位：r/s）
 */
float MotorControl_GetFeedbackVel(MotorControl *mc, MotorControlID id);

/**
 * @brief 获取单个电机的PID输出（调试用）
 * @param mc：MotorControl实例指针
 * @param id：电机ID（MOTOR_LF~MOTOR_LR）
 * @return PID输出值（对应PWM占空比：-100~100）
 */
float MotorControl_GetPIDOutput(MotorControl *mc, MotorControlID id);

#endif // MOTOR_CONTROL_H
