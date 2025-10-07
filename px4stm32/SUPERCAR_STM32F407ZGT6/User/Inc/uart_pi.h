/*
使用示例

// 示例：发送指令5给上位机
if (UartPi_SendCmd(&uart_pi_, 5)) {
    printf("UART发送指令5成功\r\n");
} else {
    printf("UART发送指令5失败\r\n");
}

// 在主循环或定时器中断中调用
void UartPi_Test(UartPi *uart_pi) {
    // 1. 接收上位机数据
    if (UartPi_Receive(uart_pi)) {
        // 2. 解析数据
        if (UartPi_ParseFrame(uart_pi)) {
            // 3. 处理解析结果
            // 处理“解码完成”指令
            if (uart_pi->is_decode_done) {
                printf("UART接收：解码完成指令\r\n");
                uart_pi->is_decode_done = false;  // 清空标志
            }
            // 处理“偏移值”数据
            if (uart_pi->offset_id != 0) {
                printf("UART接收：偏移值ID=%d，值=%.2f\r\n",
                       uart_pi->offset_id,
                       uart_pi->offset_value);
                uart_pi->offset_id = 0;  // 清空标志
            }
        }
    }
}

// 在main函数主循环中调用
while (1) {
    UartPi_Test(&uart_pi_);
    HAL_Delay(100);  // 降低CPU占用
}


*/

#ifndef UART_PI_H
#define UART_PI_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// -------------------------- 上位机帧结构定义（与ROS代码对齐） --------------------------
// 1. 解码完成帧（固定4字节）：0xAA + 0x00 + 0x01 + 0x55
#define FRAME_DECODE_DONE_LEN 4
#define FRAME_HEAD 0xAA       // 帧头
#define FRAME_TAIL 0x55       // 帧尾
#define CMD_DECODE_DONE 0x00  // 解码完成指令码
#define DATA_DECODE_DONE 0x01 // 解码完成数据

// 2. 偏移值帧（固定8字节）：0xAA + ID(1~7) + 偏移值(float,4字节) + 0x55
#define FRAME_OFFSET_DATA_LEN 8
#define ID_RED 1   // 红色标识
#define ID_GREEN 2 // 绿色标识
#define ID_BLUE 3  // 蓝色标识
#define ID_ZHU 4   // ZHU标识
#define ID_ZHUI 5  // ZHUI标识
#define ID_GU 6    // GU标识
#define ID_GRAY 7  // 灰色标识

// -------------------------- UartPi结构体（封装UART句柄与缓冲区） --------------------------
typedef struct
{
    UART_HandleTypeDef *huart; // UART句柄（如&huart2）
    uint8_t recv_buf[128];     // 接收缓冲区（与上位机一致，128字节）
    uint16_t recv_len;         // 实际接收长度
    bool is_recv_complete;     // 接收完成标志
    // 解析结果存储
    bool is_decode_done; // 解码完成指令标志
    uint8_t offset_id;   // 偏移值ID（1~7）
    float offset_value;  // 偏移值（解析后的float）
} UartPi;

// -------------------------- 函数声明 --------------------------
/**
 * @brief 初始化UartPi模块
 * @param uart_pi：UartPi实例指针
 * @param huart：UART句柄（如&huart2，需提前调用MX_USART2_UART_Init()）
 */
void UartPi_Init(UartPi *uart_pi, UART_HandleTypeDef *huart);

/**
 * @brief 发送1~11单字节指令给上位机
 * @param uart_pi：UartPi实例指针
 * @param cmd：指令（1~11，超出范围会返回错误）
 * @return true：发送成功；false：发送失败（指令无效或UART错误）
 */
bool UartPi_SendCmd(UartPi *uart_pi, uint8_t cmd);

/**
 * @brief 接收上位机数据（非阻塞，需配合中断或定时调用）
 * @param uart_pi：UartPi实例指针
 * @return true：接收成功；false：无数据或接收错误
 */
bool UartPi_Receive(UartPi *uart_pi);

/**
 * @brief 解析上位机发送的帧数据（解码完成帧/偏移值帧）
 * @param uart_pi：UartPi实例指针
 * @return true：解析到有效帧；false：无有效帧或数据错误
 */
bool UartPi_ParseFrame(UartPi *uart_pi);

/**
 * @brief 清空UartPi接收缓冲区与解析标志
 * @param uart_pi：UartPi实例指针
 */
void UartPi_ClearBuf(UartPi *uart_pi);

#endif // UART_PI_H
