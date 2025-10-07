#include "encoder_odom.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 计算轮子移动距离
static float CalculateWheelDistance(const EncoderOdom *odom, int32_t diff)
{
    if (odom == NULL)
        return 0.0f;
    // 距离 = (脉冲数 / 分辨率) * 周长
    return (float)diff / odom->encoder_resolution * (2.0f * M_PI * odom->wheel_radius);
}

void EncoderOdom_Init(EncoderOdom *odom,
                      float wheel_radius,
                      float wheel_base_width,
                      float wheel_base_length,
                      int32_t encoder_resolution)
{
    if (odom == NULL)
        return;

    // 初始化硬件参数
    odom->wheel_radius = wheel_radius;
    odom->wheel_base_width = wheel_base_width;
    odom->wheel_base_length = wheel_base_length;
    odom->encoder_resolution = encoder_resolution;

    // 重置里程计数据
    EncoderOdom_Reset(odom);
}

void EncoderOdom_UpdateFromEncoder(EncoderOdom *odom, const Encoder *encoder)
{
    if (odom == NULL || encoder == NULL)
        return;

    // 从编码器获取差分数据和时间戳
    odom->lf_diff = Encoder_GetLastLeftFrontDiff(encoder);
    odom->rf_diff = Encoder_GetLastRightFrontDiff(encoder);
    odom->rr_diff = Encoder_GetLastRightRearDiff(encoder);
    odom->lr_diff = Encoder_GetLastLeftRearDiff(encoder);
    odom->last_time = Encoder_GetLastSampleTime(encoder);
    odom->current_time = Encoder_GetCurrentSampleTime(encoder);
}

void EncoderOdom_Calculate(EncoderOdom *odom)
{
    if (odom == NULL)
        return;

    // 计算每个轮子的移动距离
    float lf_dist = CalculateWheelDistance(odom, odom->lf_diff);
    float rf_dist = CalculateWheelDistance(odom, odom->rf_diff);
    float rr_dist = CalculateWheelDistance(odom, odom->rr_diff);
    float lr_dist = CalculateWheelDistance(odom, odom->lr_diff);

    // 计算时间差
    uint32_t time_diff_ms = odom->current_time - odom->last_time;
    if (time_diff_ms == 0)
        return;

    // 计算机器人运动状态
    float avg_linear_dist = (lf_dist + rf_dist + rr_dist + lr_dist) / 4.0f;
    float left_wheel_avg = (lf_dist + lr_dist) / 2.0f;
    float right_wheel_avg = (rf_dist + rr_dist) / 2.0f;

    // 航向角变化
    float delta_theta = (right_wheel_avg - left_wheel_avg) / odom->wheel_base_width;
    odom->odom_data.theta += delta_theta;

    // 角度归一化到[-π, π]
    while (odom->odom_data.theta > M_PI)
        odom->odom_data.theta -= 2.0f * M_PI;
    while (odom->odom_data.theta < -M_PI)
        odom->odom_data.theta += 2.0f * M_PI;

    // 更新位置
    odom->odom_data.x += avg_linear_dist * cosf(odom->odom_data.theta);
    odom->odom_data.y += avg_linear_dist * sinf(odom->odom_data.theta);

    // 更新时间戳
    odom->odom_data.time = odom->current_time;
}

void EncoderOdom_Reset(EncoderOdom *odom)
{
    if (odom == NULL)
        return;

    // 重置里程计数据
    odom->odom_data.x = 0.0f;
    odom->odom_data.y = 0.0f;
    odom->odom_data.theta = 0.0f;
    odom->odom_data.time = 0;

    // 重置差分数据
    odom->lf_diff = 0;
    odom->rf_diff = 0;
    odom->rr_diff = 0;
    odom->lr_diff = 0;
    odom->last_time = 0;
    odom->current_time = 0;
}

// 数据获取接口实现
float EncoderOdom_GetX(const EncoderOdom *odom)
{
    return (odom != NULL) ? odom->odom_data.x : 0.0f;
}

float EncoderOdom_GetY(const EncoderOdom *odom)
{
    return (odom != NULL) ? odom->odom_data.y : 0.0f;
}

float EncoderOdom_GetTheta(const EncoderOdom *odom)
{
    return (odom != NULL) ? odom->odom_data.theta : 0.0f;
}

uint32_t EncoderOdom_GetTime(const EncoderOdom *odom)
{
    return (odom != NULL) ? odom->odom_data.time : 0;
}
