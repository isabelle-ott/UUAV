#include "position_control.h"
#include "math.h"
#include "stdio.h"

// 静态函数声明
static void PositionControl_CalculateWheelSpeeds(PositionControl *pc);
static void PositionControl_UpdatePosition(PositionControl *pc, float dt);
static float PositionControl_NormalizeAngle(float angle);
static float PositionControl_Clamp(float value, float min, float max);

/**
 * @brief 初始化位置控制模块
 * @param pc：PositionControl实例指针
 * @param motor_control：电机控制实例指针
 */
void PositionControl_Init(PositionControl *pc, MotorControl *motor_control)
{
    if (pc == NULL || motor_control == NULL)
    {
        return;
    }

    // 绑定电机控制实例
    pc->motor_control = motor_control;

    // 初始化底盘状态
    pc->pos_x = 0.0f;
    pc->pos_y = 0.0f;
    pc->angle = 0.0f;

    // 初始化运动参数
    pc->linear_x = 0.0f;
    pc->linear_y = 0.0f;
    pc->angular_z = 0.0f;

    // 设置速度限制
    pc->max_linear_velocity = 1.0f;      // 最大线速度1m/s
    pc->max_angular_velocity = M_PI / 2; // 最大角速度π/2 rad/s(90°/s)

    // 初始化状态
    pc->is_moving = 0;

    printf("Position control initialized\n");
}

/**
 * @brief 设置底盘目标位置
 * @param pc：PositionControl实例指针
 * @param target_x：目标x坐标(m)
 * @param target_y：目标y坐标(m)
 * @param max_speed：最大移动速度(m/s)
 */
void PositionControl_SetPosition(PositionControl *pc, float target_x, float target_y, float max_speed)
{
    if (pc == NULL)
    {
        return;
    }

    // 限制最大速度
    float limited_speed = PositionControl_Clamp(max_speed, 0.1f, pc->max_linear_velocity);

    // 计算目标位置与当前位置的差值
    float dx = target_x - pc->pos_x;
    float dy = target_y - pc->pos_y;
    float distance = sqrtf(dx * dx + dy * dy);

    if (distance < 0.01f)
    {
        // 目标位置与当前位置接近，直接停止
        PositionControl_Stop(pc);
        return;
    }

    // 计算朝向目标位置所需的角度
    float target_angle = atan2f(dy, dx);
    float angle_diff = PositionControl_NormalizeAngle(target_angle - pc->angle);

    // 先旋转到目标朝向，再直线移动
    if (fabsf(angle_diff) > 0.1f)
    {
        // 朝向未对准，先旋转
        PositionControl_SetAngle(pc, angle_diff, pc->max_angular_velocity);
    }
    else
    {
        // 朝向已对准，直线移动
        pc->linear_x = limited_speed * cosf(pc->angle);
        pc->linear_y = limited_speed * sinf(pc->angle);
        pc->angular_z = 0.0f;
        pc->is_moving = 1;
    }
}

/**
 * @brief 设置底盘目标朝向
 * @param pc：PositionControl实例指针
 * @param target_angle：目标角度(rad)，正值顺时针旋转，负值逆时针旋转
 * @param max_angular_speed：最大旋转速度(rad/s)
 */
void PositionControl_SetAngle(PositionControl *pc, float target_angle, float max_angular_speed)
{
    if (pc == NULL)
    {
        return;
    }

    // 限制最大角速度
    float limited_angular_speed = PositionControl_Clamp(fabsf(max_angular_speed), 0.1f, pc->max_angular_velocity);

    // 计算角度差并标准化
    float target_abs_angle = PositionControl_NormalizeAngle(pc->angle + target_angle);
    float angle_diff = PositionControl_NormalizeAngle(target_abs_angle - pc->angle);

    // 设置角速度
    pc->angular_z = (angle_diff > 0) ? limited_angular_speed : -limited_angular_speed;
    pc->linear_x = 0.0f;
    pc->linear_y = 0.0f;
    pc->is_moving = 1;
}

/**
 * @brief 位置控制主循环，需定时调用
 * @param pc：PositionControl实例指针
 */
void PositionControl_Loop(PositionControl *pc)
{
    if (pc == NULL || pc->motor_control == NULL)
    {
        return;
    }

    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // 计算时间间隔(s)

    if (dt >= CONTROL_LOOP_PERIOD / 1000.0f)
    {
        // 更新位置
        PositionControl_UpdatePosition(pc, dt);

        // 计算轮速
        PositionControl_CalculateWheelSpeeds(pc);

        // 重置时间戳
        last_time = current_time;
    }
}

/**
 * @brief 停止底盘运动
 * @param pc：PositionControl实例指针
 */
void PositionControl_Stop(PositionControl *pc)
{
    if (pc == NULL)
    {
        return;
    }

    // 停止所有运动
    pc->linear_x = 0.0f;
    pc->linear_y = 0.0f;
    pc->angular_z = 0.0f;
    pc->is_moving = 0;

    // 停止电机
    MotorControl_StopAll(pc->motor_control);
}

/**
 * @brief 获取当前底盘位置
 * @param pc：PositionControl实例指针
 * @param pos_x：x坐标指针
 * @param pos_y：y坐标指针
 * @param angle：朝向角度指针
 */
void PositionControl_GetPosition(PositionControl *pc, float *pos_x, float *pos_y, float *angle)
{
    if (pc == NULL || pos_x == NULL || pos_y == NULL || angle == NULL)
    {
        return;
    }

    *pos_x = pc->pos_x;
    *pos_y = pc->pos_y;
    *angle = pc->angle;
}

/**
 * @brief 计算四个轮子的目标速度
 * @param pc：PositionControl实例指针
 */
static void PositionControl_CalculateWheelSpeeds(PositionControl *pc)
{
    if (pc == NULL)
    {
        return;
    }

    double R = WHEEL_RADIUS;
    double L = WHEEL_TRACK;
    double W = WHEEL_BASE;

    // 麦轮逆运动学计算
    double front_left_target = (pc->linear_x - pc->linear_y - pc->angular_z * (L / 2 + W / 2)) / R;
    double front_right_target = (pc->linear_x + pc->linear_y + pc->angular_z * (L / 2 + W / 2)) / R;
    double back_left_target = (pc->linear_x + pc->linear_y - pc->angular_z * (L / 2 + W / 2)) / R;
    double back_right_target = (pc->linear_x - pc->linear_y + pc->angular_z * (L / 2 + W / 2)) / R;

    // 设置电机目标速度
    MotorControl_SetTargetVel(pc->motor_control, MOTOR_LF, (float)front_left_target);
    MotorControl_SetTargetVel(pc->motor_control, MOTOR_RF, (float)front_right_target);
    MotorControl_SetTargetVel(pc->motor_control, MOTOR_RR, (float)back_right_target);
    MotorControl_SetTargetVel(pc->motor_control, MOTOR_LR, (float)back_left_target);
}

/**
 * @brief 更新底盘位置
 * @param pc：PositionControl实例指针
 * @param dt：时间间隔(s)
 */
static void PositionControl_UpdatePosition(PositionControl *pc, float dt)
{
    if (pc == NULL || dt <= 0.0f)
    {
        return;
    }

    // 更新位置
    pc->pos_x += pc->linear_x * dt;
    pc->pos_y += pc->linear_y * dt;

    // 更新朝向
    pc->angle = PositionControl_NormalizeAngle(pc->angle + pc->angular_z * dt);

    // 检查是否到达目标位置（简化版，实际应用中需要更精确的判断）
    if (pc->linear_x == 0.0f && pc->linear_y == 0.0f && pc->angular_z == 0.0f)
    {
        pc->is_moving = 0;
    }
}

/**
 * @brief 标准化角度到[-π, π]
 * @param angle：输入角度(rad)
 * @return：标准化后的角度(rad)
 */
static float PositionControl_NormalizeAngle(float angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0f * M_PI;
    }
    return angle;
}

/**
 * @brief 限制值在指定范围内
 * @param value：输入值
 * @param min：最小值
 * @param max：最大值
 * @return：限制后的值
 */
static float PositionControl_Clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}
