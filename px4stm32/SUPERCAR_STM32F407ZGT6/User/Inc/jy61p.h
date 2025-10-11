#ifndef JY61P_H
#define JY61P_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "main.h"

typedef struct
{
    int16_t ax_raw;   // X轴加速度原始值（16位二进制补码）
    int16_t ay_raw;   // Y轴加速度原始值
    int16_t az_raw;   // Z轴加速度原始值
    int16_t temp_acc; // 加速度帧附带温度原始值
    float ax_g;       // 转换后X轴加速度（单位：g，1g≈9.8m/s²）
    float ay_g;       // 转换后Y轴加速度
    float az_g;       // 转换后Z轴加速度
    float temp_acc_c; // 转换后温度（单位：℃）
} JY61P_Acc;

typedef struct
{
    int16_t wx_raw;    // X轴角速度原始值（16位二进制补码）
    int16_t wy_raw;    // Y轴角速度原始值
    int16_t wz_raw;    // Z轴角速度原始值
    int16_t temp_gyro; // 角速度帧附带温度原始值
    float wx_dps;      // 转换后X轴角速度（单位：°/s）
    float wy_dps;      // 转换后Y轴角速度
    float wz_dps;      // 转换后Z轴角速度
    float temp_gyro_c; // 转换后温度（单位：℃）
} JY61P_Gyro;

typedef struct
{
    int16_t roll_raw;   // X轴滚转角原始值（16位二进制补码）
    int16_t pitch_raw;  // Y轴俯仰角原始值
    int16_t yaw_raw;    // Z轴偏航角原始值
    int16_t temp_angle; // 角度帧附带温度原始值
    float roll_deg;     // 转换后X轴滚转角（单位：°）
    float pitch_deg;    // 转换后Y轴俯仰角
    float yaw_deg;      // 转换后Z轴偏航角
    float temp_angle_c; // 转换后温度（单位：℃）
} JY61P_Angle;

typedef struct
{
    int MSH; // 毫秒高8位
    int MSL; // 毫秒低8位
    int MM;  // 分
    int SS;  // 秒
    int MS;  // 毫秒
} JY61P_Tim;

extern JY61P_Acc g_jy61p_acc;     // 全局加速度变量
extern JY61P_Gyro g_jy61p_gyro;   // 全局角速度变量
extern JY61P_Angle g_jy61p_angle; // 全局角度变量
extern JY61P_Tim g_jy61p_Tim;
void JY61P_Init(UART_HandleTypeDef *huart); // 初始化JY61P（启动USART3接收）
void JY61P_ParseData(uint8_t data);         // 解析USART3接收的单字节数据

#endif // JY61P_H
