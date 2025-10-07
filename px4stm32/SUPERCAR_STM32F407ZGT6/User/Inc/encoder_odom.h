/*
EncoderOdom_Reset(&encoder_odom_);

// 方法1：使用函数接口
float x = EncoderOdom_GetX(&encoder_odom_);
float y = EncoderOdom_GetY(&encoder_odom_);
float theta = EncoderOdom_GetTheta(&encoder_odom_);

// 方法2：使用便捷宏（直接访问）
float x = encoder_odom_x(&encoder_odom_);
float y = encoder_odom_y(&encoder_odom_);
float theta = encoder_odom_theta(&encoder_odom_);
uint32_t time = encoder_odom_time(&encoder_odom_);


*/

#ifndef ENCODER_ODOM_H
#define ENCODER_ODOM_H

#include "stm32f4xx_hal.h"
#include "encoder.h" // 包含Encoder结构体定义

// 里程计数据结构体
typedef struct
{
    float x;       // X坐标(米)
    float y;       // Y坐标(米)
    float theta;   // 航向角(弧度)
    uint32_t time; // 时间戳(ms)
} OdomData;

// 编码器里程计结构体
typedef struct
{
    // 硬件参数
    float wheel_radius;         // 轮子半径(米)
    float wheel_base_width;     // 轮距(米)
    float wheel_base_length;    // 轴距(米)
    int32_t encoder_resolution; // 编码器分辨率(线数×减速比)

    // 里程计数据
    OdomData odom_data;

    // 编码器差分数据
    int32_t lf_diff;
    int32_t rf_diff;
    int32_t rr_diff;
    int32_t lr_diff;
    uint32_t last_time;
    uint32_t current_time;
} EncoderOdom;

// 函数声明
void EncoderOdom_Init(EncoderOdom *odom,
                      float wheel_radius,
                      float wheel_base_width,
                      float wheel_base_length,
                      int32_t encoder_resolution);

// 从编码器获取数据并更新里程计
void EncoderOdom_UpdateFromEncoder(EncoderOdom *odom, const Encoder *encoder);

// 计算里程计数据
void EncoderOdom_Calculate(EncoderOdom *odom);

// 重置里程计
void EncoderOdom_Reset(EncoderOdom *odom);

// 数据获取接口
float EncoderOdom_GetX(const EncoderOdom *odom);
float EncoderOdom_GetY(const EncoderOdom *odom);
float EncoderOdom_GetTheta(const EncoderOdom *odom);
uint32_t EncoderOdom_GetTime(const EncoderOdom *odom);

// 便捷访问宏定义 - 直接暴露里程计数据
#define encoder_odom_x(odom) ((odom)->odom_data.x)
#define encoder_odom_y(odom) ((odom)->odom_data.y)
#define encoder_odom_theta(odom) ((odom)->odom_data.theta)
#define encoder_odom_time(odom) ((odom)->odom_data.time)

#endif // ENCODER_ODOM_H
