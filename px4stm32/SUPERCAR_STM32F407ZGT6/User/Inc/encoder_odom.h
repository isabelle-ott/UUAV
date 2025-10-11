// 待修改完善轴距等

#ifndef ENCODER_ODOM_H
#define ENCODER_ODOM_H

#include "stm32f4xx_hal.h"
#include "encoder.h" // 包含Encoder结构体定义

// 里程计数据结构体
typedef struct
{
    float x;       // X坐标(米) - 车右方为正
    float y;       // Y坐标(米) - 车头方向为正
    float theta;   // 航向角(弧度) - 顺时针为正，范围[-π, π]
    uint32_t time; // 时间戳(ms)
} OdomData;

// 编码器里程计结构体
typedef struct
{
    // 硬件参数
    float wheel_radius;         // 轮子半径(米) - 34mm = 0.034f
    float wheel_track;          // 轮距(米) - 左右轮间距230mm = 0.230f
    float wheel_base;           // 轴距(米) - 前后轮间距94mm = 0.094f
    int32_t encoder_resolution; // 编码器分辨率(线数×减速比)

    // 里程计数据
    OdomData odom_data;

    // 编码器差分数据（临时存储）
    int32_t lf_diff;       // 左前编码器差分（正转=车轮前进方向）
    int32_t rf_diff;       // 右前编码器差分
    int32_t rr_diff;       // 右后编码器差分
    int32_t lr_diff;       // 左后编码器差分
    uint32_t last_time;    // 上一次采样时间(ms)
    uint32_t current_time; // 当前采样时间(ms)
} EncoderOdom;

// 函数声明
void EncoderOdom_Init(EncoderOdom *odom,
                      float wheel_radius,
                      float wheel_track,
                      float wheel_base,
                      int32_t encoder_resolution);

// 从编码器获取差分数据并更新时间戳
void EncoderOdom_UpdateFromEncoder(EncoderOdom *odom, const Encoder *encoder);

// 核心：计算麦轮底盘里程计（x/y/theta）
void EncoderOdom_Calculate(EncoderOdom *odom);

// 重置里程计（归零x/y/theta）
void EncoderOdom_Reset(EncoderOdom *odom);

// 数据获取接口
float EncoderOdom_GetX(const EncoderOdom *odom);
float EncoderOdom_GetY(const EncoderOdom *odom);
float EncoderOdom_GetTheta(const EncoderOdom *odom);
uint32_t EncoderOdom_GetTime(const EncoderOdom *odom);

// 便捷访问宏定义（直接访问结构体成员，高效）
#define encoder_odom_x(odom) ((odom)->odom_data.x)
#define encoder_odom_y(odom) ((odom)->odom_data.y)
#define encoder_odom_theta(odom) ((odom)->odom_data.theta)
#define encoder_odom_time(odom) ((odom)->odom_data.time)

#endif // ENCODER_ODOM_H
