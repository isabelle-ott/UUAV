#include "encoder.h"
#include <math.h>

// -------------------------- 静态工具函数（仅内部使用） --------------------------
/**
 * @brief 计算单次采样的计数差值（基于初始值ENCODER_INIT_VAL）
 * @param current_cnt：当前定时器计数值（uint32_t）
 * @return 带符号差值（int32_t，正=正转，负=反转）
 */
static int32_t Encoder_CalculateSingleDiff(uint32_t current_cnt)
{
    return (int32_t)current_cnt - ENCODER_INIT_VAL;
}

/**
 * @brief 读取指定定时器的当前计数差值
 * @param htim：定时器句柄（非NULL）
 * @return 带符号差值（int32_t）
 */
static int32_t Encoder_GetTimerDiff(TIM_HandleTypeDef *htim)
{
    if (htim == NULL)
        return 0;
    return Encoder_CalculateSingleDiff(__HAL_TIM_GET_COUNTER(htim));
}

// -------------------------- 初始化函数实现 --------------------------
void Encoder_Init(Encoder *encoder,
                  TIM_HandleTypeDef *htim_lf,
                  TIM_HandleTypeDef *htim_rf,
                  TIM_HandleTypeDef *htim_rr,
                  TIM_HandleTypeDef *htim_lr)
{
    // 入参合法性检查（避免空指针访问）
    if (encoder == NULL || htim_lf == NULL || htim_rf == NULL || htim_rr == NULL || htim_lr == NULL)
        return;

    // 1. 绑定硬件定时器句柄
    encoder->htim_left_front = htim_lf;
    encoder->htim_right_front = htim_rf;
    encoder->htim_right_rear = htim_rr;
    encoder->htim_left_rear = htim_lr;

    // 2. 初始化数据成员
    encoder->last_lf_diff = 0;
    encoder->last_rf_diff = 0;
    encoder->last_rr_diff = 0;
    encoder->last_lr_diff = 0;

    encoder->last_sample_time = 0;
    encoder->current_sample_time = 0;

    encoder->current_lf_vel = 0.0f;
    encoder->current_rf_vel = 0.0f;
    encoder->current_rr_vel = 0.0f;
    encoder->current_lr_vel = 0.0f;

    // 3. 初始化计数器
    Encoder_ResetAllCounts(encoder);
}

// -------------------------- 核心采样函数实现 --------------------------
void Encoder_Sample(Encoder *encoder)
{
    if (encoder == NULL)
        return;

    // 1. 更新采样时间
    encoder->last_sample_time = encoder->current_sample_time;
    encoder->current_sample_time = HAL_GetTick();

    // 2. 计算时间差
    uint32_t time_diff_ms = encoder->current_sample_time - encoder->last_sample_time;
    if (time_diff_ms == 0)
        return;
    float time_diff_s = (float)time_diff_ms / 1000.0f;

    // 3. 读取当前各轮计数差值
    int32_t curr_lf_diff = Encoder_GetTimerDiff(encoder->htim_left_front);
    int32_t curr_rf_diff = Encoder_GetTimerDiff(encoder->htim_right_front);
    int32_t curr_rr_diff = Encoder_GetTimerDiff(encoder->htim_right_rear);
    int32_t curr_lr_diff = Encoder_GetTimerDiff(encoder->htim_left_rear);

    // 4. 保存本次差值
    encoder->last_lf_diff = curr_lf_diff;
    encoder->last_rf_diff = curr_rf_diff;
    encoder->last_rr_diff = curr_rr_diff;
    encoder->last_lr_diff = curr_lr_diff;

    // 5. 计算速度（公式：速度 = 计数差值 / (线数*减速比) / 时间差 → 单位：圈/秒）
    float pulses_per_circle = (float)(XIANSHU * JIANSUBI); // 每圈总脉冲数
    encoder->current_lf_vel = curr_lf_diff / pulses_per_circle / time_diff_s;
    encoder->current_rf_vel = curr_rf_diff / pulses_per_circle / time_diff_s;
    encoder->current_rr_vel = curr_rr_diff / pulses_per_circle / time_diff_s;
    encoder->current_lr_vel = curr_lr_diff / pulses_per_circle / time_diff_s;

    // 6. 重置计数器
    Encoder_ResetAllCounts(encoder);
}

// -------------------------- 速度获取接口实现 --------------------------
float Encoder_GetLeftFrontVel(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->current_lf_vel : 0.0f;
}

float Encoder_GetRightFrontVel(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->current_rf_vel : 0.0f;
}

float Encoder_GetRightRearVel(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->current_rr_vel : 0.0f;
}

float Encoder_GetLeftRearVel(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->current_lr_vel : 0.0f;
}

// -------------------------- 采样时间获取接口实现 --------------------------
uint32_t Encoder_GetLastSampleTime(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->last_sample_time : 0;
}

uint32_t Encoder_GetCurrentSampleTime(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->current_sample_time : 0;
}

uint32_t Encoder_GetSampleTimeDiff(const Encoder *encoder)
{
    if (encoder == NULL)
        return 0;
    return encoder->current_sample_time - encoder->last_sample_time; // 自动处理回卷
}

// -------------------------- 计数差值获取接口实现 --------------------------
int32_t Encoder_GetLastLeftFrontDiff(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->last_lf_diff : 0;
}

int32_t Encoder_GetLastRightFrontDiff(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->last_rf_diff : 0;
}

int32_t Encoder_GetLastRightRearDiff(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->last_rr_diff : 0;
}

int32_t Encoder_GetLastLeftRearDiff(const Encoder *encoder)
{
    return (encoder != NULL) ? encoder->last_lr_diff : 0;
}

// -------------------------- 计数器重置接口实现 --------------------------
void Encoder_ResetLeftFrontCount(Encoder *encoder)
{
    if (encoder == NULL || encoder->htim_left_front == NULL)
        return;
    __HAL_TIM_SET_COUNTER(encoder->htim_left_front, ENCODER_INIT_VAL);
    // encoder->last_lf_diff = 0;
}

void Encoder_ResetRightFrontCount(Encoder *encoder)
{
    if (encoder == NULL || encoder->htim_right_front == NULL)
        return;
    __HAL_TIM_SET_COUNTER(encoder->htim_right_front, ENCODER_INIT_VAL);
    // encoder->last_rf_diff = 0;
}

void Encoder_ResetRightRearCount(Encoder *encoder)
{
    if (encoder == NULL || encoder->htim_right_rear == NULL)
        return;
    __HAL_TIM_SET_COUNTER(encoder->htim_right_rear, ENCODER_INIT_VAL);
    // encoder->last_rr_diff = 0;
}

void Encoder_ResetLeftRearCount(Encoder *encoder)
{
    if (encoder == NULL || encoder->htim_left_rear == NULL)
        return;
    __HAL_TIM_SET_COUNTER(encoder->htim_left_rear, ENCODER_INIT_VAL);
    // encoder->last_lr_diff = 0;
}

void Encoder_ResetAllCounts(Encoder *encoder)
{
    if (encoder == NULL)
        return;
    Encoder_ResetLeftFrontCount(encoder);
    Encoder_ResetRightFrontCount(encoder);
    Encoder_ResetRightRearCount(encoder);
    Encoder_ResetLeftRearCount(encoder);
}
