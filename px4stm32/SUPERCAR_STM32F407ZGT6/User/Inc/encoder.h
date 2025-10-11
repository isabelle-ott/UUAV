/*
主循环测试：
    lf_diff = Encoder_GetLastLeftFrontDiff(&encoder_);

    last_sample_t = Encoder_GetLastSampleTime(&encoder_);
    curr_sample_t = Encoder_GetCurrentSampleTime(&encoder_);
    time_diff = Encoder_GetSampleTimeDiff(&encoder_);

    lf_vel = Encoder_GetLeftFrontVel(&encoder_);
*/

#pragma once

#include "main.h"
#include "tim.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"

// -------------------------- 硬件参数宏定义 --------------------------
#define XIANSHU 500            // 编码器线数
#define JIANSUBI 34            // 电机减速比
#define ENCODER_INIT_VAL 32767 // 16位计数器初始值
#define ENCODER_MAX_VAL 65535  // 16位计数器最大值

// -------------------------- 编码器结构体定义 --------------------------

typedef struct
{
    // 1. 硬件绑定：定时器句柄
    TIM_HandleTypeDef *htim_left_front;
    TIM_HandleTypeDef *htim_right_front;
    TIM_HandleTypeDef *htim_right_rear;
    TIM_HandleTypeDef *htim_left_rear;

    // 2. 采样数据：与上次差值
    int32_t last_lf_diff;
    int32_t last_rf_diff;
    int32_t last_rr_diff;
    int32_t last_lr_diff;

    // 3. 时间数据：采样时刻（单位：ms，基于HAL_GetTick()）
    uint32_t last_sample_time;    // 上次采样时间
    uint32_t current_sample_time; // 当前采样时间

    // 4. 速度数据：各轮实时速度（单位：圈/秒）
    float current_lf_vel;
    float current_rf_vel;
    float current_rr_vel;
    float current_lr_vel;
} Encoder;

// -------------------------- 核心接口函数声明 --------------------------
/**
 * @brief 编码器初始化
 * @param encoder：编码器结构体实例指针
 * @param htim_lf：左前轮定时器句柄
 * @param htim_rf：右前轮定时器句柄
 * @param htim_rr：右后轮定时器句柄
 * @param htim_lr：左后轮定时器句柄
 */
void Encoder_Init(Encoder *encoder,
                  TIM_HandleTypeDef *htim_lf,
                  TIM_HandleTypeDef *htim_rf,
                  TIM_HandleTypeDef *htim_rr,
                  TIM_HandleTypeDef *htim_lr);

/**
 * @brief 编码器采样核心函数（中断调用）
 * @功能：1. 记录采样时间 2. 计算计数差值 3. 计算速度 4. 重置计数器防溢出
 * @param encoder：编码器结构体实例指针（非NULL）
 */
void Encoder_Sample(Encoder *encoder);

// -------------------------- 速度获取接口 --------------------------
/**
 * @brief 获取轮子速度（单位：圈/秒）
 * @param encoder：编码器结构体实例指针（非NULL）
 * @return 速度值（float，正=正转，负=反转）
 */
float Encoder_GetLeftFrontVel(const Encoder *encoder);
float Encoder_GetRightFrontVel(const Encoder *encoder);
float Encoder_GetRightRearVel(const Encoder *encoder);
float Encoder_GetLeftRearVel(const Encoder *encoder);

// -------------------------- 采样时间获取接口 --------------------------
/**
 * @brief 获取上次采样时间（单位：ms，基于HAL_GetTick()）
 * @param encoder：编码器结构体实例指针（非NULL）
 * @return 上次采样时刻（uint32_t）
 */
uint32_t Encoder_GetLastSampleTime(const Encoder *encoder);

/**
 * @brief 获取当前采样时间（单位：ms，基于HAL_GetTick()）
 * @param encoder：编码器结构体实例指针（非NULL）
 * @return 当前采样时刻（uint32_t）
 */
uint32_t Encoder_GetCurrentSampleTime(const Encoder *encoder);

/**
 * @brief 获取两次采样的时间差（单位：ms）
 * @param encoder：编码器结构体实例指针（非NULL）
 * @return 时间差（uint32_t，自动处理HAL_GetTick()回卷）
 */
uint32_t Encoder_GetSampleTimeDiff(const Encoder *encoder);

// -------------------------- 计数差值获取接口 --------------------------
/**
 * @brief 获取轮子上次采样的计数差值（用于计算位移）
 * @param encoder：编码器结构体实例指针（非NULL）
 * @return 计数差值（int32_t，正=正转，负=反转）
 */
int32_t Encoder_GetLastLeftFrontDiff(const Encoder *encoder);
int32_t Encoder_GetLastRightFrontDiff(const Encoder *encoder);
int32_t Encoder_GetLastRightRearDiff(const Encoder *encoder);
int32_t Encoder_GetLastLeftRearDiff(const Encoder *encoder);

// -------------------------- 计数器重置接口 --------------------------
/**
 * @brief 重置轮子编码器计数器（设为ENCODER_INIT_VAL）
 * @param encoder：编码器结构体实例指针（非NULL）
 */
void Encoder_ResetLeftFrontCount(Encoder *encoder);
void Encoder_ResetRightFrontCount(Encoder *encoder);
void Encoder_ResetRightRearCount(Encoder *encoder);
void Encoder_ResetLeftRearCount(Encoder *encoder);

/**
 * @brief 重置所有编码器计数器（设为ENCODER_INIT_VAL）
 * @param encoder：编码器结构体实例指针（非NULL）
 */
void Encoder_ResetAllCounts(Encoder *encoder);
