// #pragma once
// #include "stm32f4xx_hal.h"
// #include "encoder_odom.h"

// #ifdef __cplusplus
// extern "C"
// {
// #endif

//     // IMU初始化
//     void IMU_Init(void);

//     // 获取IMU原始数据
//     void IMU_GetRawData(float *accel, float *gyro);

//     // 获取IMU里程计数据
//     void IMU_GetOdomData(OdomData *imu_odom);

//     // IMU数据更新（需要在中断中调用）
//     void IMU_Update(void);

// #ifdef __cplusplus
// }
// #endif