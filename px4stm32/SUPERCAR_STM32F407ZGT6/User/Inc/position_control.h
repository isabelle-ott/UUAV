#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "motor_control.h"
#include "stdint.h"

// 底盘参数配置（根据实际硬件设置）
#define WHEEL_RADIUS 0.034f     // 车轮半径(m)
#define WHEEL_TRACK 0.230f      // 左右轮距(m)
#define WHEEL_BASE 0.094f       // 前后轮距(m)
#define CONTROL_LOOP_PERIOD 20U // 控制周期(ms)，建议20ms(50Hz)

// 位置控制结构体
typedef struct
{
    MotorControl *motor_control; // 电机控制实例指针

    // 底盘状态
    float pos_x; // x坐标(m)
    float pos_y; // y坐标(m)
    float angle; // 朝向角度(rad)，车头为y正方向

    // 运动参数
    float linear_x;  // x方向线速度(m/s)
    float linear_y;  // y方向线速度(m/s)
    float angular_z; // 角速度(rad/s)

    // 最大运动速度限制
    float max_linear_velocity;  // 最大线速度(m/s)
    float max_angular_velocity; // 最大角速度(rad/s)

    // 控制标志
    uint8_t is_moving; // 是否正在运动
} PositionControl;

// -------------------------- 核心接口函数声明 --------------------------
/**
 * @brief 位置控制模块初始化
 * @param pc：PositionControl实例指针
 * @param motor_control：已初始化的电机控制实例
 */
void PositionControl_Init(PositionControl *pc, MotorControl *motor_control);

/**
 * @brief 设置底盘目标位置
 * @param pc：PositionControl实例指针
 * @param target_x：目标x坐标(m)
 * @param target_y：目标y坐标(m)
 * @param max_speed：最大移动速度(m/s)
 */
void PositionControl_SetPosition(PositionControl *pc, float target_x, float target_y, float max_speed);

/**
 * @brief 设置底盘目标朝向
 * @param pc：PositionControl实例指针
 * @param target_angle：目标角度(rad)，正值顺时针旋转，负值逆时针旋转
 * @param max_angular_speed：最大旋转速度(rad/s)
 */
void PositionControl_SetAngle(PositionControl *pc, float target_angle, float max_angular_speed);

/**
 * @brief 位置控制主循环，需定时调用
 * @param pc：PositionControl实例指针
 */
void PositionControl_Loop(PositionControl *pc);

/**
 * @brief 停止底盘运动
 * @param pc：PositionControl实例指针
 */
void PositionControl_Stop(PositionControl *pc);

/**
 * @brief 获取当前底盘位置
 * @param pc：PositionControl实例指针
 * @param pos_x：x坐标指针
 * @param pos_y：y坐标指针
 * @param angle：朝向角度指针
 */
void PositionControl_GetPosition(PositionControl *pc, float *pos_x, float *pos_y, float *angle);

#endif // POSITION_CONTROL_H
