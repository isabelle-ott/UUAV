// #include "imu_odom.h"
// #include <math.h>

// // IMU状态变量
// static float imu_theta = 0.0f;     // IMU计算的航向角
// static float imu_x = 0.0f;         // IMU计算的位置x
// static float imu_y = 0.0f;         // IMU计算的位置y
// static uint32_t last_imu_time = 0; // 上次IMU更新时间
// static float accel_bias[3] = {0};  // 加速度计偏置
// static float gyro_bias[3] = {0};   // 陀螺仪偏置

// // IMU初始化
// void IMU_Init(void)
// {
//     imu_theta = 0.0f;
//     imu_x = 0.0f;
//     imu_y = 0.0f;
//     last_imu_time = HAL_GetTick();

//     // 初始化偏置（实际应用中需要校准）
//     for (int i = 0; i < 3; i++)
//     {
//         accel_bias[i] = 0.0f;
//         gyro_bias[i] = 0.0f;
//     }
// }

// // 模拟获取IMU原始数据（实际需要根据你的IMU驱动修改）
// void IMU_GetRawData(float *accel, float *gyro)
// {
//     // 这里应该是你的IMU读取代码
//     // 示例：模拟IMU数据
//     accel[0] = 0.01f + ((rand() % 100) - 50) * 0.001f; // ax
//     accel[1] = 0.02f + ((rand() % 100) - 50) * 0.001f; // ay
//     accel[2] = 9.81f + ((rand() % 100) - 50) * 0.001f; // az

//     gyro[0] = 0.05f + ((rand() % 100) - 50) * 0.001f;  // gx
//     gyro[1] = -0.03f + ((rand() % 100) - 50) * 0.001f; // gy
//     gyro[2] = 0.1f + ((rand() % 100) - 50) * 0.001f;   // gz
// }

// // IMU数据更新（在定时中断中调用）
// void IMU_Update(void)
// {
//     float accel[3], gyro[3];
//     IMU_GetRawData(accel, gyro);

//     uint32_t current_time = HAL_GetTick();
//     float dt = (current_time - last_imu_time) / 1000.0f;
//     if (dt <= 0)
//         return;

//     // 去除偏置
//     for (int i = 0; i < 3; i++)
//     {
//         accel[i] -= accel_bias[i];
//         gyro[i] -= gyro_bias[i];
//     }

//     // 积分陀螺仪数据得到角度（简化版）
//     imu_theta += gyro[2] * dt; // 使用Z轴角速度

//     // 限制角度范围 [-π, π]
//     while (imu_theta > M_PI)
//         imu_theta -= 2 * M_PI;
//     while (imu_theta < -M_PI)
//         imu_theta += 2 * M_PI;

//     last_imu_time = current_time;
// }

// // 获取IMU里程计数据
// void IMU_GetOdomData(OdomData *imu_odom)
// {
//     if (imu_odom != nullptr)
//     {
//         imu_odom->x = imu_x;
//         imu_odom->y = imu_y;
//         imu_odom->theta = imu_theta;
//         imu_odom->time = last_imu_time;
//     }
// }