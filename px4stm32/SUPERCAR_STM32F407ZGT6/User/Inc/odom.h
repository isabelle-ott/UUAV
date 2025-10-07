// #pragma once
// #include "encoder_odom.h"
// #include "imu_odom.h"
// #include "stm32f4xx_hal.h"

// #ifdef __cplusplus
// extern "C"
// {
// #endif

//     // 融合里程计数据结构
//     typedef struct
//     {
//         float x;             // 融合X坐标(米)
//         float y;             // 融合Y坐标(米)
//         float theta;         // 融合航向角(弧度)
//         uint32_t time;       // 时间戳(ms)
//         float covariance[9]; // 协方差矩阵
//     } FusedOdomData;

//     // 融合初始化
//     void OdomFusion_Init(void);

//     // 获取融合后的里程计数据
//     void GetFusedOdomData(FusedOdomData *fused_odom);

//     // 获取原始传感器数据（用于调试）
//     void GetRawSensorData(OdomData *encoder_odom, OdomData *imu_odom);

//     // 融合算法更新（在定时中断中调用）
//     void OdomFusion_Update(void);

// #ifdef __cplusplus
// }
// #endif