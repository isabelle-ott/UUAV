#include "uart_pi.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// -------------------------- 初始化UartPi --------------------------
void UartPi_Init(UartPi *uart_pi, UART_HandleTypeDef *huart)
{
    if (uart_pi == NULL || huart == NULL)
    {
        return;
    }
    // 绑定UART句柄
    uart_pi->huart = huart;
    // 初始化缓冲区与标志
    memset(uart_pi->recv_buf, 0, sizeof(uart_pi->recv_buf));
    uart_pi->recv_len = 0;
    uart_pi->is_recv_complete = false;
    // 初始化解析结果
    uart_pi->is_decode_done = false;
    uart_pi->offset_id = 0;
    uart_pi->offset_value = 0.0f;
}

// -------------------------- 发送1~11单字节指令 --------------------------
bool UartPi_SendCmd(UartPi *uart_pi, uint8_t cmd)
{
    if (uart_pi == NULL || uart_pi->huart == NULL)
    {
        return false;
    }
    // 检查指令范围（1~11，与上位机一致）
    if (cmd < 1 || cmd > 11)
    {
        return false;
    }
    // 阻塞发送（超时100ms，避免死等）
    HAL_StatusTypeDef status = HAL_UART_Transmit(
        uart_pi->huart,
        &cmd,
        1,
        100 // 超时时间（ms）
    );
    return (status == HAL_OK);
}

// -------------------------- 接收上位机数据 --------------------------
bool UartPi_Receive(UartPi *uart_pi)
{
    if (uart_pi == NULL || uart_pi->huart == NULL)
    {
        return false;
    }
    // 清空之前的接收数据
    UartPi_ClearBuf(uart_pi);

    // 非阻塞接收（最多接收128字节，超时50ms，与上位机线程延时对齐）
    HAL_StatusTypeDef status = HAL_UART_Receive(
        uart_pi->huart,
        uart_pi->recv_buf,
        sizeof(uart_pi->recv_buf),
        50 // 超时时间（ms），与上位机sleep(50ms)匹配
    );

    if (status == HAL_OK)
    {
        // 记录接收长度（实际接收128字节，或上位机发送的实际长度）
        uart_pi->recv_len = sizeof(uart_pi->recv_buf);
        uart_pi->is_recv_complete = true;
        return true;
    }
    else if (status == HAL_TIMEOUT)
    {
        // 超时：可能无数据，或数据未发完，不处理
        return false;
    }
    else
    {
        // 接收错误（如帧错误、溢出）
        UartPi_ClearBuf(uart_pi);
        return false;
    }
}

// -------------------------- 解析上位机帧数据 --------------------------
bool UartPi_ParseFrame(UartPi *uart_pi)
{
    if (uart_pi == NULL || !uart_pi->is_recv_complete || uart_pi->recv_len == 0)
    {
        return false;
    }

    bool has_valid_frame = false;
    // 遍历接收缓冲区，查找帧头+帧尾（支持多帧连续接收）
    for (uint16_t i = 0; i < uart_pi->recv_len; i++)
    {
        // 1. 检查帧头（0xAA）
        if (uart_pi->recv_buf[i] != FRAME_HEAD)
        {
            continue;
        }

        // 2. 解析“解码完成帧”（4字节：0xAA 0x00 0x01 0x55）
        if ((i + FRAME_DECODE_DONE_LEN) <= uart_pi->recv_len)
        {
            if (uart_pi->recv_buf[i + 1] == CMD_DECODE_DONE &&
                uart_pi->recv_buf[i + 2] == DATA_DECODE_DONE &&
                uart_pi->recv_buf[i + 3] == FRAME_TAIL)
            {
                uart_pi->is_decode_done = true;
                has_valid_frame = true;
                i += FRAME_DECODE_DONE_LEN - 1; // 跳过已解析的帧
                continue;
            }
        }

        // 3. 解析“偏移值帧”（8字节：0xAA + ID + 4字节float + 0x55）
        if ((i + FRAME_OFFSET_DATA_LEN) <= uart_pi->recv_len)
        {
            if (uart_pi->recv_buf[i + FRAME_OFFSET_DATA_LEN - 1] == FRAME_TAIL)
            {
                uint8_t id = uart_pi->recv_buf[i + 1];
                // 检查ID范围（1~7）
                if (id >= ID_RED && id <= ID_GRAY)
                {
                    uart_pi->offset_id = id;
                    // float转4字节（小端模式，与上位机reinterpret_cast对齐）
                    memcpy(&uart_pi->offset_value, &uart_pi->recv_buf[i + 2], 4);
                    has_valid_frame = true;
                    i += FRAME_OFFSET_DATA_LEN - 1; // 跳过已解析的帧
                    continue;
                }
            }
        }
    }

    // 解析完成后清空接收标志（避免重复解析）
    uart_pi->is_recv_complete = false;
    return has_valid_frame;
}

// -------------------------- 清空缓冲区与标志 --------------------------
void UartPi_ClearBuf(UartPi *uart_pi)
{
    if (uart_pi == NULL)
    {
        return;
    }
    memset(uart_pi->recv_buf, 0, sizeof(uart_pi->recv_buf));
    uart_pi->recv_len = 0;
    uart_pi->is_recv_complete = false;
    // 解析标志不清空（需用户自行处理后清空）
}
