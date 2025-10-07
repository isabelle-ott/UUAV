#include "motor_control.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/**
 * @brief 检查电机ID合法性
 * @param id：电机ID
 * @return 合法返回1，非法返回0
 */
static uint8_t MotorControl_CheckID(MotorControlID id)
{
    return (id >= MOTOR_LF && id < MOTOR_NUM) ? 1 : 0;
}

/**
 * @brief 从编码器获取单个电机的反馈速度
 * @param mc：MotorControl实例指针
 * @param id：电机ID
 * @return 反馈速度（单位：r/s），非法输入返回0.0f
 */
static float MotorControl_GetEncoderVel(MotorControl *mc, MotorControlID id)
{
    if (mc == NULL || mc->encoder == NULL)
        return 0.0f;

    // 根据电机ID获取对应编码器速度（与Encoder模块接口对齐）
    switch (id)
    {
    case MOTOR_LF:
        return Encoder_GetLeftFrontVel(mc->encoder);
    case MOTOR_RF:
        return Encoder_GetRightFrontVel(mc->encoder);
    case MOTOR_RR:
        return Encoder_GetRightRearVel(mc->encoder);
    case MOTOR_LR:
        return Encoder_GetLeftRearVel(mc->encoder);
    default:
        return 0.0f;
    }
}

/**
 * @brief 初始化单个电机的PID控制器
 * @param mc：MotorControl实例指针
 * @param id：电机ID
 */
static void MotorControl_InitSinglePID(MotorControl *mc, MotorControlID id)
{
    if (!MotorControl_CheckID(id))
        return;

    // 初始化增量式PID（速度控制优先选择增量式，抗干扰性强）
    PID_Init(&mc->pid[id], PID_VELOCITY,
             mc->pid_kp, mc->pid_ki, mc->pid_kd,
             mc->pid_integral_limit, mc->pid_output_limit);
}

void MotorControl_Init(MotorControl *mc,
                       Motor *motor,
                       Encoder *encoder,
                       float pid_kp,
                       float pid_ki,
                       float pid_kd,
                       float pid_integral_limit,
                       float pid_output_limit,
                       uint32_t control_freq)
{
    if (mc == NULL || motor == NULL || encoder == NULL)
        return;

    // 1. 绑定外部依赖实例
    mc->motor = motor;
    mc->encoder = encoder;

    // 2. 初始化PID参数（确保参数合法）
    mc->pid_kp = fabsf(pid_kp);
    mc->pid_ki = fabsf(pid_ki);
    mc->pid_kd = fabsf(pid_kd);
    mc->pid_integral_limit = fabsf(pid_integral_limit);
    mc->pid_output_limit = fabsf(pid_output_limit);
    // 限制输出范围（避免超出电机PWM控制范围）
    if (mc->pid_output_limit > 100.0f)
        mc->pid_output_limit = 100.0f;
    else if (mc->pid_output_limit < 10.0f)
        mc->pid_output_limit = 10.0f; // 最小输出限幅10，防止电机堵转

    // 3. 初始化控制频率（50~200Hz合理范围）
    mc->control_freq = (control_freq < 50) ? 50 : (control_freq > 200) ? 200
                                                                       : control_freq;

    // 4. 初始化4个电机的PID控制器
    for (MotorControlID id = MOTOR_LF; id < MOTOR_NUM; id++)
    {
        MotorControl_InitSinglePID(mc, id);
    }

    // 5. 初始状态：停止所有电机
    MotorControl_StopAll(mc);
}

void MotorControl_SetTargetVel(MotorControl *mc, MotorControlID id, float target_vel)
{
    if (!MotorControl_CheckID(id) || mc == NULL)
        return;

    // 设置PID目标速度（直接传递给对应电机的PID控制器）
    PID_SetTarget(&mc->pid[id], target_vel);
}

void MotorControl_Loop(MotorControl *mc)
{
    if (mc == NULL || mc->motor == NULL || mc->encoder == NULL)
        return;

    // 遍历所有电机，执行PID闭环控制
    for (MotorControlID id = MOTOR_LF; id < MOTOR_NUM; id++)
    {
        // 1. 步骤1：获取编码器反馈速度
        float feedback_vel = MotorControl_GetEncoderVel(mc, id);

        // 2. 步骤2：更新PID反馈值
        PID_SetFeedback(&mc->pid[id], feedback_vel);

        // 3. 步骤3：执行PID计算，得到PWM输出（-100~100）
        float pid_output = PID_Calculate(&mc->pid[id]);

        // 4. 步骤4：将PID输出传递给电机，控制转速
        switch (id)
        {
        case MOTOR_LF:
            Motor_SetLeftFrontVel(mc->motor, pid_output);
            break;
        case MOTOR_RF:
            Motor_SetRightFrontVel(mc->motor, pid_output);
            break;
        case MOTOR_RR:
            Motor_SetRightRearVel(mc->motor, pid_output);
            break;
        case MOTOR_LR:
            Motor_SetLeftRearVel(mc->motor, pid_output);
            break;
        default:
            break;
        }
    }
}

void MotorControl_StopAll(MotorControl *mc)
{
    if (mc == NULL || mc->motor == NULL)
        return;

    // 停止所有电机PWM输出
    Motor_StopAll(mc->motor);

    // 重置所有电机的PID控制器（清空积分，避免下次启动冲击）
    for (MotorControlID id = MOTOR_LF; id < MOTOR_NUM; id++)
    {
        PID_Reset(&mc->pid[id]);
        // 重置目标速度为0
        PID_SetTarget(&mc->pid[id], 0.0f);
    }
}

float MotorControl_GetFeedbackVel(MotorControl *mc, MotorControlID id)
{
    if (!MotorControl_CheckID(id))
        return 0.0f;

    // 返回编码器反馈速度（与PID反馈值一致）
    return PID_GetFeedback(&mc->pid[id]); // 需在pid.h中补充PID_GetFeedback接口
}

float MotorControl_GetPIDOutput(MotorControl *mc, MotorControlID id)
{
    if (!MotorControl_CheckID(id) || mc == NULL)
        return 0.0f;

    // 返回PID当前输出值
    return PID_GetOutput(&mc->pid[id]);
}
