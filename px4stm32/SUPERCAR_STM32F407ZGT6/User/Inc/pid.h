#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// PID控制器模式枚举
typedef enum
{
    PID_POSITION = 0, // 位置式PID（适用于位置控制，如base_position）
    PID_VELOCITY = 1  // 增量式PID（适用于速度控制，如轮子速度）
} PID_Mode;

// PID控制器结构体
typedef struct
{
    PID_Mode mode;        // PID模式（位置/增量）
    float kp;             // 比例系数
    float ki;             // 积分系数
    float kd;             // 微分系数
    float integral_limit; // 积分限幅
    float output_limit;   // 输出限幅

    // 位置式PID变量
    float target;        // 目标值
    float feedback;      // 反馈值
    float integral;      // 积分累加值
    float last_feedback; // 上一次反馈值
    float output;        // 当前输出值

    // 时间戳（计算微分时间间隔）
    uint32_t last_time;    // 上一次计算时间（ms）
    uint32_t current_time; // 当前计算时间（ms）
} PID_Controller;

// -------------------------- PID核心接口 --------------------------
/**
 * @brief PID控制器初始化
 * @param pid：PID实例指针（如&lf_pid）
 * @param mode：PID模式（PID_POSITION/PID_VELOCITY）
 * @param kp/ki/kd：PID参数
 * @param integral_limit：积分限幅
 * @param output_limit：输出限幅
 */
void PID_Init(PID_Controller *pid,
              PID_Mode mode,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_limit);

/**
 * @brief 设置PID目标值
 * @param pid：PID实例指针
 * @param target：目标值（如速度目标值、位置目标值）
 */
void PID_SetTarget(PID_Controller *pid, float target);

/**
 * @brief 设置PID反馈值（需在计算前调用）
 * @param pid：PID实例指针
 * @param feedback：反馈值（如编码器速度反馈、位置反馈）
 */
void PID_SetFeedback(PID_Controller *pid, float feedback);

/**
 * @brief PID计算（核心函数，返回当前输出值）
 * @param pid：PID实例指针
 * @return 计算后的输出值（已限幅）
 */
float PID_Calculate(PID_Controller *pid);

/**
 * @brief 重置PID控制器（清空积分、重置时间戳）
 * @param pid：PID实例指针
 */
void PID_Reset(PID_Controller *pid);

// -------------------------- 数据获取接口（补充PID_GetFeedback） --------------------------
/**
 * @brief 获取PID当前反馈值
 * @param pid：PID实例指针
 * @return 当前反馈值
 */
static inline float PID_GetFeedback(const PID_Controller *pid)
{
    return (pid != NULL) ? pid->feedback : 0.0f;
}

/**
 * @brief 获取PID当前输出值
 * @param pid：PID实例指针
 * @return 当前输出值
 */
static inline float PID_GetOutput(const PID_Controller *pid)
{
    return (pid != NULL) ? pid->output : 0.0f;
}

#endif // PID_H
