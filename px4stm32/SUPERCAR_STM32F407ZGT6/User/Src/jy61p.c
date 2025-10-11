#include "jy61p.h"

// 接收缓冲区（静态局部变量，仅内部使用，不对外暴露）
uint8_t s_rx_buffer[11];
static uint8_t s_rx_cnt = 0;
static UART_HandleTypeDef *s_huart = NULL;

// 帧解析函数
static void JY61P_ParseFrame(void)
{
    // 第一步：校验帧头（必须为0x55）
    if (s_rx_buffer[0] != 0x55)
    {
        s_rx_cnt = 0; // 帧头错误，重置计数器
        return;
    }

    // 第二步：校验和验证（前10字节求和，与第11字节比较）
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; i++) // 修正：计算前10字节的和
    {
        sum += s_rx_buffer[i];
    }
    if (sum != s_rx_buffer[10])
    { // 校验和不匹配，丢弃当前帧
        s_rx_cnt = 0;
        return;
    }

    // 第三步：按数据类型解析
    switch (s_rx_buffer[1])
    {
    case 0x51: // 加速度帧
        g_jy61p_acc.ax_raw = (s_rx_buffer[3] << 8) | s_rx_buffer[2];
        g_jy61p_acc.ay_raw = (s_rx_buffer[5] << 8) | s_rx_buffer[4];
        g_jy61p_acc.az_raw = (s_rx_buffer[7] << 8) | s_rx_buffer[6];
        g_jy61p_acc.temp_acc = (s_rx_buffer[9] << 8) | s_rx_buffer[8];
        g_jy61p_acc.ax_g = ((float)g_jy61p_acc.ax_raw) / 32768.0f * 16.0f * 9.8f;
        g_jy61p_acc.ay_g = ((float)g_jy61p_acc.ay_raw) / 32768.0f * 16.0f * 9.8f;
        g_jy61p_acc.az_g = ((float)g_jy61p_acc.az_raw) / 32768.0f * 16.0f * 9.8f;
        g_jy61p_acc.temp_acc_c = (float)g_jy61p_acc.temp_acc / 100.0f;
        break;

    case 0x52: // 角速度帧
        g_jy61p_gyro.wx_raw = (s_rx_buffer[3] << 8) | s_rx_buffer[2];
        g_jy61p_gyro.wy_raw = (s_rx_buffer[5] << 8) | s_rx_buffer[4];
        g_jy61p_gyro.wz_raw = (s_rx_buffer[7] << 8) | s_rx_buffer[6];
        g_jy61p_gyro.temp_gyro = (s_rx_buffer[9] << 8) | s_rx_buffer[8];
        g_jy61p_gyro.wx_dps = (float)g_jy61p_gyro.wx_raw / 32768.0f * 2000.0f;
        g_jy61p_gyro.wy_dps = (float)g_jy61p_gyro.wy_raw / 32768.0f * 2000.0f;
        g_jy61p_gyro.wz_dps = (float)g_jy61p_gyro.wz_raw / 32768.0f * 2000.0f;
        g_jy61p_gyro.temp_gyro_c = (float)g_jy61p_gyro.temp_gyro / 100.0f;
        break;

    case 0x53: // 角度帧
        g_jy61p_angle.roll_raw = (s_rx_buffer[3] << 8) | s_rx_buffer[2];
        g_jy61p_angle.pitch_raw = (s_rx_buffer[5] << 8) | s_rx_buffer[4];
        g_jy61p_angle.yaw_raw = (s_rx_buffer[7] << 8) | s_rx_buffer[6];
        g_jy61p_angle.temp_angle = (s_rx_buffer[9] << 8) | s_rx_buffer[8];
        g_jy61p_angle.roll_deg = (float)g_jy61p_angle.roll_raw / 32768.0f * 180.0f;
        g_jy61p_angle.pitch_deg = (float)g_jy61p_angle.pitch_raw / 32768.0f * 180.0f;
        g_jy61p_angle.yaw_deg = (float)g_jy61p_angle.yaw_raw / 32768.0f * 180.0f;
        g_jy61p_angle.temp_angle_c = (float)g_jy61p_angle.temp_angle / 100.0f;
        break;

    default:
        break;
    }

    s_rx_cnt = 0;
}

// 4. JY61P初始化
void JY61P_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
        return;
    s_huart = huart;

    // 启动USART3接收中断
    static uint8_t temp_data = 0;
    HAL_UART_Receive_IT(s_huart, &temp_data, 1);
}

// 5. 数据接收处理
void JY61P_ParseData(uint8_t data)
{
    if (s_rx_cnt == 0)
    {
        if (data == 0x55)
        {
            s_rx_buffer[s_rx_cnt++] = data;
        }
        return;
    }
    else if (s_rx_cnt == 1)
    {
        if (data == 0x51 || data == 0x52 || data == 0x53)
        {
            s_rx_buffer[s_rx_cnt++] = data;
        }
        else
        {
            s_rx_cnt = 0;
            if (data == 0x55)
            {
                s_rx_buffer[s_rx_cnt++] = data;
            }
        }
        return;
    }
    else if (s_rx_cnt < 11)
    {
        s_rx_buffer[s_rx_cnt++] = data;
    }

    if (s_rx_cnt == 11)
    {
        JY61P_ParseFrame();
    }
}
