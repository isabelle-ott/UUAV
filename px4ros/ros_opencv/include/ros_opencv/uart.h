#ifndef UART_H
#define UART_H

#include <string>
#include <serial/serial.h>

class UARTCommunicator
{
private:
    // 串口对象
    serial::Serial serial_port_;

    // 串口参数
    std::string port_;
    int baud_rate_;

public:
    // 构造函数和析构函数
    UARTCommunicator(const std::string &port = "/dev/ttyUSB0", int baud_rate = 115200);
    ~UARTCommunicator();

    // 设置串口参数
    void setPort(const std::string &port);
    void setBaudRate(int baud_rate);

    // 初始化并打开串口
    bool open();

    // 关闭串口
    void close();

    // 串口发送数据
    // 参数: data - 要发送的数据缓冲区, len - 数据长度
    // 返回值: 实际发送的字节数, 失败返回-1
    int send(const unsigned char *data, size_t len);

    // 串口接收数据
    // 参数: buffer - 接收数据的缓冲区, max_len - 缓冲区最大长度
    // 返回值: 实际接收的字节数, 失败返回-1
    int receive(unsigned char *buffer, size_t max_len);

    // 检查串口是否打开
    bool isOpen() const;
};

#endif // UART_H
