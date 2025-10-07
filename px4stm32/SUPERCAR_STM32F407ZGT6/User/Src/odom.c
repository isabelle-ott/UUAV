// #include "odom.h"
// #include <math.h>
// #include <string.h>

// // 融合状态
// static FusedOdomData fused_odom = {0};
// static OdomData last_encoder_odom = {0};
// static OdomData last_imu_odom = {0};
// static uint32_t last_fusion_time = 0;

// // 简单互补滤波器参数
// static const float ALPHA = 0.98f; // 陀螺仪权重
// static const float BETA = 0.02f;  // 编码器权重

// // 融合初始化
// void OdomFusion_Init(void)
// {
//     memset(&fused_odom, 0, sizeof(FusedOdomData));
//     memset(&last_encoder_odom, 0, sizeof(OdomData));
//     memset(&last_imu_odom, 0, sizeof(OdomData));

//     // 初始化协方差矩阵
//     for (int i = 0; i < 9; i++)
//     {
//         fused_odom.covariance[i] = (i % 4 == 0) ? 1.0f : 0.0f; // 对角矩阵
//     }

//     last_fusion_time = HAL_GetTick();

//     // 初始化传感器
//     IMU_Init();
// }

// // 互补滤波器融合角度
// static float ComplementaryFilter(float encoder_theta, float imu_theta, float gyro_z, float dt)
// {
//     // 互补滤波器：使用陀螺仪积分得到角度，用编码器角度进行校正
//     static float last_fused_theta = 0.0f;

//     // 陀螺仪积分
//     float gyro_theta = last_fused_theta + gyro_z * dt;

//     // 互补滤波
//     float fused_theta = ALPHA * gyro_theta + BETA * encoder_theta;

//     // 限制角度范围
//     while (fused_theta > M_PI)
//         fused_theta -= 2 * M_PI;
//     while (fused_theta < -M_PI)
//         fused_theta += 2 * M_PI;

//     last_fused_theta = fused_theta;
//     return fused_theta;
// }

// // 位置融合（主要使用编码器，IMU用于验证）
// static void FusePosition(const OdomData *encoder, const OdomData *imu, FusedOdomData *fused)
// {
//     // 位置主要依赖编码器（更准确）
//     fused->x = encoder->x;
//     fused->y = encoder->y;

//     // 简单的协方差更新
//     fused->covariance[0] = 0.1f; // x方差
//     fused->covariance[4] = 0.1f; // y方差
// }

// // 融合算法更新
// void OdomFusion_Update(void)
// {
//     // 更新IMU数据
//     IMU_Update();

//     // 获取传感器数据
//     OdomData encoder_odom, imu_odom;
//     encoder_odom_.GetEncoderOdomData(&encoder_odom);
//     IMU_GetOdomData(&imu_odom);

//     uint32_t current_time = HAL_GetTick();
//     float dt = (current_time - last_fusion_time) / 1000.0f;
//     if (dt <= 0)
//         return;

//     // 获取IMU原始数据用于角度融合
//     float accel[3], gyro[3];
//     IMU_GetRawData(accel, gyro);

//     // 角度融合（互补滤波）
//     fused_odom.theta = ComplementaryFilter(encoder_odom.theta, imu_odom.theta, gyro[2], dt);

//     // 位置融合
//     FusePosition(&encoder_odom, &imu_odom, &fused_odom);

//     // 更新时间
//     fused_odom.time = current_time;

//     // 保存数据用于下次融合
//     last_encoder_odom = encoder_odom;
//     last_imu_odom = imu_odom;
//     last_fusion_time = current_time;
// }

// // 获取融合后的里程计数据
// void GetFusedOdomData(FusedOdomData *fused_odom_ptr)
// {
//     if (fused_odom_ptr != nullptr)
//     {
//         *fused_odom_ptr = fused_odom;
//     }
// }

// // 获取原始传感器数据（用于调试）
// void GetRawSensorData(OdomData *encoder_odom, OdomData *imu_odom)
// {
//     if (encoder_odom != nullptr)
//     {
//         *encoder_odom = last_encoder_odom;
//     }
//     if (imu_odom != nullptr)
//     {
//         *imu_odom = last_imu_odom;
//     }
// }