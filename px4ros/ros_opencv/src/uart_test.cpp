#include "ros_opencv/uart.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>

// 测试数据：简单的命令帧结构
const uint8_t TEST_FRAME[] = {0xAA, 0x01, 0x02, 0x03, 0x04, 0x55};
const size_t TEST_FRAME_LEN = sizeof(TEST_FRAME) / sizeof(TEST_FRAME[0]);

// 接收线程函数
void receiveThread(UARTCommunicator &uart)
{
    uint8_t recvBuffer[128];
    std::cout << "Receive thread started" << std::endl;

    while (true)
    {
        int recvLen = uart.receive(recvBuffer, sizeof(recvBuffer));

        if (recvLen > 0)
        {
            std::cout << "\nReceived " << recvLen << " bytes: ";
            for (int i = 0; i < recvLen; ++i)
            {
                printf("0x%02X ", recvBuffer[i]);
            }
            std::cout << std::endl;
        }
        else if (recvLen < 0)
        {
            std::cerr << "Receive error occurred" << std::endl;
            // 短暂延时避免错误时CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

int main(int argc, char *argv[])
{
    // 默认串口参数，可通过命令行参数修改
    std::string port = "/dev/ttyUSB0";
    int baudRate = 115200;

    // 解析命令行参数
    if (argc >= 2)
    {
        port = argv[1];
    }
    if (argc >= 3)
    {
        baudRate = std::stoi(argv[2]);
    }

    std::cout << "Starting UART test node" << std::endl;
    std::cout << "Using port: " << port << ", Baud rate: " << baudRate << std::endl;

    // 创建串口通信对象
    UARTCommunicator uart(port, baudRate);

    // 打开串口
    if (!uart.open())
    {
        std::cerr << "Failed to open UART port! Exiting..." << std::endl;
        return -1;
    }

    // 启动接收线程
    std::thread recvThread(receiveThread, std::ref(uart));
    recvThread.detach(); // 分离线程，独立运行

    // 主循环：定时发送测试数据
    try
    {
        while (true)
        {
            // 发送测试帧
            int sendLen = uart.send(TEST_FRAME, TEST_FRAME_LEN);

            if (sendLen > 0)
            {
                std::cout << "\nSent " << sendLen << " bytes: ";
                for (size_t i = 0; i < TEST_FRAME_LEN; ++i)
                {
                    printf("0x%02X ", TEST_FRAME[i]);
                }
                std::cout << std::endl;
            }
            else
            {
                std::cerr << "Failed to send data" << std::endl;
            }

            // 每2秒发送一次
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Main loop exception: " << e.what() << std::endl;
    }

    // 关闭串口
    uart.close();
    return 0;
}
