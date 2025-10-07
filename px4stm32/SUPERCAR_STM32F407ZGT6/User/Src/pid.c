#include "pid.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/**
 * @brief 计算时间间隔（ms），防止溢出
 * @param last：上一次时间
 * @param current：当前时间
 * @return 时间间隔（ms），最大1000ms（避免异常大的微分）
 */
static uint32_t PID_GetTimeDiff(uint32_t last, uint32_t current)
{
    return (current >= last) ? (current - last) : (UINT32_MAX - last + current + 1);
}

void PID_Init(PID_Controller *pid,
              PID_Mode mode,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_limit)
{
    if (pid == NULL)
        return;

    // 初始化PID模式与参数
    pid->mode = mode;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = fabsf(integral_limit); // 确保限幅为正
    pid->output_limit = fabsf(output_limit);     // 确保限幅为正

    // 重置PID状态变量
    PID_Reset(pid);
}

void PID_SetTarget(PID_Controller *pid, float target)
{
    if (pid != NULL)
        pid->target = target;
}

void PID_SetFeedback(PID_Controller *pid, float feedback)
{
    if (pid != NULL)
    {
        pid->last_feedback = pid->feedback; // 保存上一次反馈值
        pid->feedback = feedback;           // 更新当前反馈值
    }
}

float PID_Calculate(PID_Controller *pid)
{
    if (pid == NULL)
        return 0.0f;

    // 1. 更新时间戳，计算时间间隔（dt，单位：s）
    pid->last_time = pid->current_time;
    pid->current_time = HAL_GetTick();
    uint32_t dt_ms = PID_GetTimeDiff(pid->last_time, pid->current_time);
    float dt = (dt_ms > 1000) ? 1.0f : (dt_ms / 1000.0f); // 限制最大dt为1s

    // 2. 计算偏差（目标值 - 反馈值）
    float error = pid->target - pid->feedback;

    // 3. 根据PID模式计算输出
    if (pid->mode == PID_POSITION)
    {
        // 位置式PID：output = kp*error + ki*∫error*dt + kd*d(error)/dt
        // 积分项（带限幅，防止积分饱和）
        pid->integral += error * dt;
        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        else if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;

        // 微分项（用反馈值的变化量，避免目标值突变导致的微分冲击）
        float derivative = (pid->feedback - pid->last_feedback) / dt;

        // 计算位置式输出
        pid->output = pid->kp * error + pid->ki * pid->integral - pid->kd * derivative;
    }
    else if (pid->mode == PID_VELOCITY)
    {
        // 增量式PID：Δoutput = kp*(error - last_error) + ki*error*dt + kd*(error - 2*last_error + pre_last_error)
        static float last_error = 0.0f; // 上一次偏差
        float delta_error = error - last_error;

        // 计算增量
        float delta_output = pid->kp * delta_error + pid->ki * error * dt + pid->kd * (delta_error - (last_error - (pid->target - pid->last_feedback)));

        // 增量累加（带限幅）
        pid->output += delta_output;
        if (pid->output > pid->output_limit)
            pid->output = pid->output_limit;
        else if (pid->output < -pid->output_limit)
            pid->output = -pid->output_limit;

        // 更新上一次偏差
        last_error = error;
    }

    // 4. 输出限幅（双重保障）
    if (pid->output > pid->output_limit)
        pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit)
        pid->output = -pid->output_limit;

    return pid->output;
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL)
        return;

    // 重置目标值、反馈值
    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->last_feedback = 0.0f;

    // 重置积分、输出
    pid->integral = 0.0f;
    pid->output = 0.0f;

    // 重置时间戳
    pid->last_time = HAL_GetTick();
    pid->current_time = pid->last_time;
}
