#include "encoder_odom.h"
#include "stm32f4xx_hal.h"
#include <math.h>

// 定义π（若未定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 计算单个轮子的移动距离（带方向）
 * @param odom：EncoderOdom实例指针
 * @param diff：编码器差分（正=轮子前进方向，负=后退）
 * @return 轮子移动距离（米，正=前进，负=后退）
 */
static float CalculateWheelDistance(const EncoderOdom *odom, int32_t diff)
{
    if (odom == NULL)
        return 0.0f;
    // 距离 = (编码器差分 / 分辨率) * 车轮周长（差分符号决定方向）
    return (float)diff / odom->encoder_resolution * (2.0f * M_PI * odom->wheel_radius);
}

/**
 * @brief 初始化麦轮里程计（绑定硬件参数）
 * @param odom：EncoderOdom实例指针
 * @param wheel_radius：车轮半径（米，34mm=0.034f）
 * @param wheel_track：轮距（米，230mm=0.230f）
 * @param wheel_base：轴距（米，94mm=0.094f）
 * @param encoder_resolution：编码器分辨率（线数×减速比，如17000）
 */
void EncoderOdom_Init(EncoderOdom *odom,
                      float wheel_radius,
                      float wheel_track,
                      float wheel_base,
                      int32_t encoder_resolution)
{
    if (odom == NULL)
        return;

    // 初始化麦轮硬件参数（关键！必须与实际硬件一致）
    odom->wheel_radius = wheel_radius;
    odom->wheel_track = wheel_track;
    odom->wheel_base = wheel_base;
    odom->encoder_resolution = encoder_resolution;

    // 重置里程计数据（x/y/theta归零）
    EncoderOdom_Reset(odom);
}

/**
 * @brief 从编码器获取差分数据和时间戳（桥梁函数）
 * @param odom：EncoderOdom实例指针
 * @param encoder：Encoder实例指针（已采样完成的编码器数据）
 */
void EncoderOdom_UpdateFromEncoder(EncoderOdom *odom, const Encoder *encoder)
{
    if (odom == NULL || encoder == NULL)
        return;

    // 1. 获取4个轮子的编码器差分（需确保Encoder结构体的差分方向正确）
    odom->lf_diff = Encoder_GetLastLeftFrontDiff(encoder);  // 左前差分
    odom->rf_diff = Encoder_GetLastRightFrontDiff(encoder); // 右前差分
    odom->rr_diff = Encoder_GetLastRightRearDiff(encoder);  // 右后差分
    odom->lr_diff = Encoder_GetLastLeftRearDiff(encoder);   // 左后差分

    // 2. 获取编码器采样时间戳（用于计算时间差）
    odom->last_time = Encoder_GetLastSampleTime(encoder);       // 上一次采样时间
    odom->current_time = Encoder_GetCurrentSampleTime(encoder); // 当前采样时间
}

/**
 * @brief 核心：计算麦轮底盘的x/y/theta（基于正运动学）
 * @param odom：EncoderOdom实例指针
 */
void EncoderOdom_Calculate(EncoderOdom *odom)
{
    if (odom == NULL)
        return;

    // 1. 计算时间差（避免除零，最小1ms）
    uint32_t time_diff_ms = odom->current_time - odom->last_time;
    if (time_diff_ms < 1)
        time_diff_ms = 1;
    float dt = (float)time_diff_ms / 1000.0f; // 转换为秒

    // 2. 计算4个轮子的移动距离（带方向）
    float lf_dist = CalculateWheelDistance(odom, odom->lf_diff); // 左前轮距离
    float rf_dist = CalculateWheelDistance(odom, odom->rf_diff); // 右前轮距离
    float rr_dist = CalculateWheelDistance(odom, odom->rr_diff); // 右后轮距离
    float lr_dist = CalculateWheelDistance(odom, odom->lr_diff); // 左后轮距离

    // 3. 基于麦轮正运动学，计算底盘瞬时速度（关键公式！）
    float R = odom->wheel_radius;                          // 车轮半径
    float L_plus_W = odom->wheel_track + odom->wheel_base; // 轮距+轴距

    // 3.1 X方向速度（车右方为正）
    float Vx = (lf_dist + rf_dist + lr_dist + rr_dist) * R / (4.0f * dt);
    // 3.2 Y方向速度（车头方向为正）
    float Vy = (-lf_dist + rf_dist + lr_dist - rr_dist) * R / (4.0f * dt);
    // 3.3 角速度（顺时针为正，rad/s）
    float Wz = (-lf_dist + rf_dist - lr_dist + rr_dist) * R / (2.0f * L_plus_W * dt);

    // 4. 积分计算x/y/theta（基于底盘速度）
    float current_theta = odom->odom_data.theta; // 当前航向角

    // 4.1 位置更新（小角度近似：cos(theta)≈1，sin(theta)≈theta，减少计算量）
    odom->odom_data.x += (Vx * cosf(current_theta) - Vy * sinf(current_theta)) * dt;
    odom->odom_data.y += (Vx * sinf(current_theta) + Vy * cosf(current_theta)) * dt;

    // 4.2 航向角更新（累加角速度×时间差）
    odom->odom_data.theta += Wz * dt;

    // 5. 航向角归一化（确保范围在[-π, π]，避免角度溢出）
    while (odom->odom_data.theta > M_PI)
        odom->odom_data.theta -= 2.0f * M_PI;
    while (odom->odom_data.theta < -M_PI)
        odom->odom_data.theta += 2.0f * M_PI;

    // 6. 更新里程计时间戳
    odom->odom_data.time = odom->current_time;
}

/**
 * @brief 重置里程计（x/y/theta归零，差分数据清空）
 * @param odom：EncoderOdom实例指针
 */
void EncoderOdom_Reset(EncoderOdom *odom)
{
    if (odom == NULL)
        return;

    // 里程计数据归零
    odom->odom_data.x = 0.0f;
    odom->odom_data.y = 0.0f;
    odom->odom_data.theta = 0.0f;
    odom->odom_data.time = 0;

    // 编码器差分数据清空
    odom->lf_diff = 0;
    odom->rf_diff = 0;
    odom->rr_diff = 0;
    odom->lr_diff = 0;
    odom->last_time = 0;
    odom->current_time = 0;
}

// -------------------------- 数据获取接口实现 --------------------------
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
