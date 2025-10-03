#include "ros_opencv/uart.h"
#include <iostream>
#include <string.h>

UARTCommunicator::UARTCommunicator(const std::string &port, int baud_rate)
    : port_(port), baud_rate_(baud_rate) {}

UARTCommunicator::~UARTCommunicator()
{
    close();
}

void UARTCommunicator::setPort(const std::string &port)
{
    port_ = port;
}

void UARTCommunicator::setBaudRate(int baud_rate)
{
    baud_rate_ = baud_rate;
}

bool UARTCommunicator::open()
{
    try
    {
        // 如果已打开则先关闭
        if (serial_port_.isOpen())
        {
            serial_port_.close();
        }

        // 配置串口参数
        serial_port_.setPort(port_);
        serial_port_.setBaudrate(baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port_.setTimeout(timeout);
        serial_port_.setParity(serial::parity_none);
        serial_port_.setStopbits(serial::stopbits_one);
        serial_port_.setBytesize(serial::eightbits);

        // 打开串口
        serial_port_.open();
        return serial_port_.isOpen();
    }
    catch (serial::IOException &e)
    {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
        return false;
    }
}

void UARTCommunicator::close()
{
    if (serial_port_.isOpen())
    {
        try
        {
            serial_port_.close();
        }
        catch (serial::IOException &e)
        {
            std::cerr << "Error closing serial port: " << e.what() << std::endl;
        }
    }
}

int UARTCommunicator::send(const unsigned char *data, size_t len)
{
    if (!serial_port_.isOpen())
    {
        std::cerr << "Serial port not open" << std::endl;
        return -1;
    }

    try
    {
        return serial_port_.write(data, len);
    }
    catch (serial::IOException &e)
    {
        std::cerr << "Serial write error: " << e.what() << std::endl;
        return -1;
    }
}

int UARTCommunicator::receive(unsigned char *buffer, size_t max_len)
{
    if (!serial_port_.isOpen())
    {
        std::cerr << "Serial port not open" << std::endl;
        return -1;
    }

    try
    {
        return serial_port_.read(buffer, max_len);
    }
    catch (serial::IOException &e)
    {
        std::cerr << "Serial read error: " << e.what() << std::endl;
        return -1;
    }
}

bool UARTCommunicator::isOpen() const
{
    return serial_port_.isOpen();
}
