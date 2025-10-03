// color_erea_detect把偏离位置发到 /red /blue /green /ZHU /ZHUI /GU /gray
// state根据if判断选择订阅哪个并告诉下位机
// 测试:下位机发送数值1～11测试状态机
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <cstring>
#include "ros_opencv/uart.h"

using namespace std;

// -------------------------- 状态机与全局变量定义 --------------------------
enum State
{
    WAITDECODE, // 起始状态
    DECODE,     // 收到1 进入DECODE 发解码给下位机
    WAIT2,      // 收到2 进入WAIT2
    PAI,        // 收到3 进入PAI 根据data[0]需要的颜色的大致位置 发相对数值给下位机
    WEITIAO1,   // 收到4 进入微调1 发data[0]最大色块中心相对位置给下位机
    WAIT3,      // 收到5 下位机完成微调 进入WAIT3
    DA,         // 收到6 进入DA 根据data[1]需要的颜色的大致位置 发相对数值给下位机
    WEITIAO2,   // 收到7 进入微调2 发data[1]最大色块中心相对位置给下位机
    WAIT4,      // 收到8 下位机完成微调 进入WAIT4
    JIU,        // 收到9 进入JIU 根据data[2]需要的面积排序的大致位置 发相对数值给下位机
    WEITIAO3,   // 收到10 进入微调3 发data[2]最大色块中心相对位置给下位机
    OVER        // 收到11 下位机完成微调 进入OVER
};

// 存储拆分后的3位数字（百位data[0]、十位data[1]、个位data[2]）
int data[3] = {0, 0, 0};
// 偏移数据存储（全局变量，回调函数更新）
float red_offset = 0.0f;
float green_offset = 0.0f;
float blue_offset = 0.0f;
float zhu_offset = 0.0f;
float zhui_offset = 0.0f;
float gu_offset = 0.0f;
float gray_offset = 0.0f;

// 串口接收的下位机指令（全局变量，接收线程更新）
int uart_recv_cmd = 0;
// 线程安全互斥锁（保护全局变量读写）
mutex data_mutex, cmd_mutex;

#define p 0.1
#define d 0.1
#define j 0.1

// -------------------------- ROS回调函数（更新偏移数据） --------------------------
void redCallback(const std_msgs::Float32::ConstPtr &msg)
{
    lock_guard<mutex> lock(data_mutex);
    red_offset = msg->data;
    ROS_INFO("Received /red offset: %.2f", red_offset);
}

void greenCallback(const std_msgs::Float32::ConstPtr &msg)
{
    lock_guard<mutex> lock(data_mutex);
    green_offset = msg->data;
    ROS_INFO("Received /green offset: %.2f", green_offset);
}

void blueCallback(const std_msgs::Float32::ConstPtr &msg)
{
    lock_guard<mutex> lock(data_mutex);
    blue_offset = msg->data;
    ROS_INFO("Received /blue offset: %.2f", blue_offset);
}

void zhuCallback(const std_msgs::Float32::ConstPtr &msg)
{
    lock_guard<mutex> lock(data_mutex);
    zhu_offset = msg->data;
    ROS_INFO("Received /ZHU offset: %.2f", zhu_offset);
}

void zhuiCallback(const std_msgs::Float32::ConstPtr &msg)
{
    lock_guard<mutex> lock(data_mutex);
    zhui_offset = msg->data;
    ROS_INFO("Received /ZHUI offset: %.2f", zhui_offset);
}

void guCallback(const std_msgs::Float32::ConstPtr &msg)
{
    lock_guard<mutex> lock(data_mutex);
    gu_offset = msg->data;
    ROS_INFO("Received /GU offset: %.2f", gu_offset);
}

void grayCallback(const std_msgs::Float32::ConstPtr &msg)
{
    lock_guard<mutex> lock(data_mutex);
    gray_offset = msg->data;
    ROS_INFO("Received /gray offset: %.2f", gray_offset);
}

// 解码回调函数（处理/decode_info话题的3位整数）
void decodeCallback(const std_msgs::Int32::ConstPtr &msg)
{
    int num = msg->data;
    lock_guard<mutex> lock(data_mutex);

    if (num >= 100 && num <= 999)
    {
        data[0] = num / 100;       // 百位
        data[1] = (num / 10) % 10; // 十位
        data[2] = num % 10;        // 个位
        ROS_INFO("Received decode number: %d -> Split: [%d, %d, %d]", num, data[0], data[1], data[2]);
    }
    else
    {
        ROS_WARN("Received invalid decode number: %d (must be 3-digit)", num);
    }
}

// -------------------------- 串口接收线程 --------------------------
void uartReceiveThread(UARTCommunicator &uart)
{
    uint8_t recv_buf[128] = {0}; // 串口接收缓冲区
    ROS_INFO("UART receive thread started");

    while (ros::ok())
    {
        // 仅传缓冲区和最大长度
        int recv_len = uart.receive(recv_buf, sizeof(recv_buf));

        if (recv_len > 0)
        {
            lock_guard<mutex> lock(cmd_mutex);
            // 解析下位机单字节指令（1~11）
            for (int i = 0; i < recv_len; ++i)
            {
                uint8_t cmd = recv_buf[i];
                if (cmd >= 1 && cmd <= 11)
                {
                    uart_recv_cmd = static_cast<int>(cmd);
                    ROS_INFO("UART received cmd: %d (0x%02X)", uart_recv_cmd, cmd);
                }
                else
                {
                    ROS_WARN("UART received invalid cmd: 0x%02X (must be 1~11)", cmd);
                }
            }
            // 清空缓冲区，避免残留数据干扰
            memset(recv_buf, 0, sizeof(recv_buf));
        }
        else if (recv_len < 0)
        {
            ROS_ERROR("UART receive error (code: %d)", recv_len);
            this_thread::sleep_for(chrono::milliseconds(100)); // 错误时延时，降低CPU占用
        }

        // 正常循环延时，避免线程空转（关键：防止CPU使用率过高）
        this_thread::sleep_for(chrono::milliseconds(50));
    }
    ROS_INFO("UART receive thread exited");
}

// -------------------------- 串口发送函 --------------------------
// 发送解码完成指令（固定帧：0xAA + 0x00 + 0x01 + 0x55）
bool sendDecodeDone(UARTCommunicator &uart)
{
    const uint8_t frame[] = {0xAA, 0x00, 0x01, 0x55}; // 帧头+指令+数据+帧尾
    int send_len = uart.send(frame, sizeof(frame));
    if (send_len == sizeof(frame))
    {
        ROS_INFO("UART sent decode done (frame: 0xAA 0x00 0x01 0x55)");
        return true;
    }
    else
    {
        ROS_ERROR("UART send decode done failed (sent: %d bytes)", send_len);
        return false;
    }
}

// 发送偏移值（帧结构：0xAA + 颜色/灰色标识 + 偏移值（float转4字节） + 0x55）
// 标识定义：1=红色,2=绿色,3=蓝色,4=ZHU,5=ZHUI,6=GU,7=gray
bool sendOffsetData(UARTCommunicator &uart, int id, float offset)
{
    if (id < 1 || id > 7)
    {
        ROS_ERROR("Invalid offset ID: %d (must be 1~7)", id);
        return false;
    }

    // 构建帧（帧头+ID+4字节偏移值+帧尾）
    uint8_t frame[8] = {0};
    frame[0] = 0xAA;                     // 帧头
    frame[1] = static_cast<uint8_t>(id); // 颜色/灰色标识
    // float转4字节（小端模式，需与下位机一致）
    uint8_t *offset_bytes = reinterpret_cast<uint8_t *>(&offset);
    for (int i = 0; i < 4; ++i)
    {
        frame[2 + i] = offset_bytes[i];
    }
    frame[6] = 0x55; // 帧尾

    int send_len = uart.send(frame, sizeof(frame));
    if (send_len == sizeof(frame))
    {
        ROS_INFO("UART sent offset (ID: %d, value: %.2f) | Frame: 0xAA 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x55",
                 id, offset, frame[1], frame[2], frame[3], frame[4], frame[5]);
        return true;
    }
    else
    {
        ROS_ERROR("UART send offset failed (ID: %d, sent: %d bytes)", id, send_len);
        return false;
    }
}

// -------------------------- 主函数（状态机逻辑，保持不变） --------------------------
int main(int argc, char **argv)
{
    // 1. 初始化ROS节点
    ros::init(argc, argv, "decode_processor");
    ros::NodeHandle nh;
    State state_ = WAITDECODE;

    // 2. 订阅ROS话题
    ros::Subscriber decode_sub = nh.subscribe("/decode_info", 10, decodeCallback);
    ros::Subscriber red_sub = nh.subscribe("/red", 10, redCallback);
    ros::Subscriber green_sub = nh.subscribe("/green", 10, greenCallback);
    ros::Subscriber blue_sub = nh.subscribe("/blue", 10, blueCallback);
    ros::Subscriber zhu_sub = nh.subscribe("/ZHU", 10, zhuCallback);
    ros::Subscriber zhui_sub = nh.subscribe("/ZHUI", 10, zhuiCallback);
    ros::Subscriber gu_sub = nh.subscribe("/GU", 10, guCallback);
    ros::Subscriber gray_sub = nh.subscribe("/gray", 10, grayCallback);

    // 3. 初始化串口（默认参数，可通过ROS参数修改）
    string uart_port;
    int uart_baud;
    nh.param<string>("/uart/port", uart_port, "/dev/ttyUSB0"); // 默认串口（如USB转串口）
    nh.param<int>("/uart/baud", uart_baud, 115200);            // 默认波特率（需与下位机一致）
    UARTCommunicator uart(uart_port, uart_baud);

    // 打开串口
    if (!uart.open())
    {
        ROS_FATAL("Failed to open UART port: %s (baud: %d)", uart_port.c_str(), uart_baud);
        return -1;
    }
    ROS_INFO("UART initialized successfully (port: %s, baud: %d)", uart_port.c_str(), uart_baud);

    // 4. 启动串口接收线程（分离线程，独立运行）
    thread recv_thread(uartReceiveThread, ref(uart));
    recv_thread.detach();

    // 5. 状态机主循环
    ros::Rate rate(10); // 10Hz循环频率（平衡响应速度与资源占用）
    while (ros::ok())
    {
        // 读取串口指令（线程安全：加锁防止数据竞争）
        int current_cmd = 0;
        {
            lock_guard<mutex> lock(cmd_mutex);
            current_cmd = uart_recv_cmd;
            uart_recv_cmd = 0; // 读取后清空指令，避免重复触发状态切换
        }

        // 状态机核心逻辑
        switch (state_)
        {
        case WAITDECODE:
            ROS_INFO_THROTTLE(3, "Current State: WAITDECODE (wait cmd 1 from lower device)");
            if (current_cmd == 1)
            {
                ROS_INFO("Switch to DECODE state (received cmd 1)");
                state_ = DECODE;
            }
            break;

        case DECODE:
            ROS_INFO_THROTTLE(3, "Current State: DECODE (processing decode data)");
            // 检查是否有有效解码数据，有则发送"解码完成"指令给下位机
            {
                lock_guard<mutex> lock(data_mutex);
                if (data[0] != 0 || data[1] != 0 || data[2] != 0)
                {
                    sendDecodeDone(uart);
                }
            }
            // 收到下位机指令2，切换到WAIT2状态
            if (current_cmd == 2)
            {
                ROS_INFO("Switch to WAIT2 state (received cmd 2)");
                state_ = WAIT2;
            }
            break;

        case WAIT2:
            ROS_INFO_THROTTLE(1, "Current State: WAIT2 (wait cmd 3 from lower device)");
            if (current_cmd == 3)
            {
                ROS_INFO("Switch to PAI state (received cmd 3)");
                state_ = PAI;
            }
            break;

        case PAI:
            ROS_INFO_THROTTLE(1, "Current State: PAI (send data[0] offset to lower device)");
            {
                lock_guard<mutex> lock(data_mutex);
                // 根据data[0]发送对应颜色偏移（1=红,2=绿,3=蓝）
                if (data[0] == 1)
                {
                    sendOffsetData(uart, 1, red_offset); // 红色：ID=1
                }
                else if (data[0] == 2)
                {
                    sendOffsetData(uart, 2, green_offset); // 绿色：ID=2
                }
                else if (data[0] == 3)
                {
                    sendOffsetData(uart, 3, blue_offset); // 蓝色：ID=3
                }
                else
                {
                    ROS_WARN("Invalid data[0] value: %d (must be 1~3)", data[0]);
                }
            }
            // 收到下位机指令4，切换到微调1状态
            if (current_cmd == 4)
            {
                ROS_INFO("Switch to WEITIAO1 state (received cmd 4)");
                state_ = WEITIAO1;
            }
            break;

        case WEITIAO1:
            ROS_INFO_THROTTLE(1, "Current State: WEITIAO1 (send data[0] fine offset)");
            {
                lock_guard<mutex> lock(data_mutex);
                // 微调阶段发送与PAI一致的颜色偏移
                if (data[0] == 1)
                {
                    sendOffsetData(uart, 1, red_offset);
                }
                else if (data[0] == 2)
                {
                    sendOffsetData(uart, 2, green_offset);
                }
                else if (data[0] == 3)
                {
                    sendOffsetData(uart, 3, blue_offset);
                }
                else
                {
                    ROS_WARN("Invalid data[0] value: %d (must be 1~3)", data[0]);
                }
            }
            // 收到下位机指令5，切换到WAIT3
            if (current_cmd == 5)
            {
                ROS_INFO("Switch to WAIT3 state (received cmd 5)");
                state_ = WAIT3;
            }
            break;

        case WAIT3:
            ROS_INFO_THROTTLE(1, "Current State: WAIT3 (wait cmd 6 from lower device)");
            if (current_cmd == 6)
            {
                ROS_INFO("Switch to DA state (received cmd6)");
                state_ = DA;
            }
            break;

        case DA:
            ROS_INFO_THROTTLE(1, "Current State: DA (send data[1] offset to lower device)");
            {
                lock_guard<mutex> lock(data_mutex);
                // 根据data[1]发送对应颜色偏移（1=红,2=绿,3=蓝）
                if (data[1] == 1)
                {
                    sendOffsetData(uart, 1, red_offset);
                }
                else if (data[1] == 2)
                {
                    sendOffsetData(uart, 2, green_offset);
                }
                else if (data[1] == 3)
                {
                    sendOffsetData(uart, 3, blue_offset);
                }
                else
                {
                    ROS_WARN("Invalid data[1] value: %d (must be 1~3)", data[1]);
                }
            }
            // 收到下位机指令7，切换到WEITIAO2
            if (current_cmd == 7)
            {
                ROS_INFO("Switch to WEITIAO2 state (received cmd 7)");
                state_ = WEITIAO2;
            }
            break;

        case WEITIAO2:
            ROS_INFO_THROTTLE(1, "Current State: WEITIAO2 (send data[1] fine offset)");
            {
                lock_guard<mutex> lock(data_mutex);
                // 微调阶段发送与DA一致的颜色偏移
                if (data[1] == 1)
                {
                    sendOffsetData(uart, 1, red_offset);
                }
                else if (data[1] == 2)
                {
                    sendOffsetData(uart, 2, green_offset);
                }
                else if (data[1] == 3)
                {
                    sendOffsetData(uart, 3, blue_offset);
                }
                else
                {
                    ROS_WARN("Invalid data[1] value: %d (must be 1~3)", data[1]);
                }
            }
            // 收到下位机指令8，切换到WAIT4
            if (current_cmd == 8)
            {
                ROS_INFO("Switch to WAIT4 state (received cmd 8)");
                state_ = WAIT4;
            }
            break;

        case WAIT4:
            ROS_INFO_THROTTLE(1, "Current State: WAIT4 (wait cmd 9 from lower device)");
            if (current_cmd == 9)
            {
                ROS_INFO("Switch to JIU state (received cmd 9)");
                state_ = JIU;
            }
            break;

        case JIU:
            ROS_INFO_THROTTLE(1, "Current State: JIU (send data[2] offset to lower device)");
            {
                lock_guard<mutex> lock(data_mutex);
                // 根据data[2]发送对应灰色偏移（1=ZHU,2=ZHUI,3=GU）
                if (data[2] == 1)
                {
                    sendOffsetData(uart, 4, zhu_offset); // ZHU对应ID=4
                }
                else if (data[2] == 2)
                {
                    sendOffsetData(uart, 5, zhui_offset); // ZHUI对应ID=5
                }
                else if (data[2] == 3)
                {
                    sendOffsetData(uart, 6, gu_offset); // GU对应ID=6
                }
                else
                {
                    ROS_WARN("Invalid data[2] value: %d (must be 1~3)", data[2]);
                }
            }
            // 收到下位机指令10，切换到WEITIAO3
            if (current_cmd == 10)
            {
                ROS_INFO("Switch to WEITIAO3 state (received cmd 10)");
                state_ = WEITIAO3;
            }
            break;

        case WEITIAO3:
            ROS_INFO_THROTTLE(1, "Current State: WEITIAO3 (send gray fine offset)");
            {
                lock_guard<mutex> lock(data_mutex);
                sendOffsetData(uart, 7, gray_offset); // gray对应ID=7
            }
            if (current_cmd == 11)
            {
                ROS_INFO("Switch to OVER state (received cmd 11, task completed)");
                state_ = OVER;
            }
            break;

        case OVER:
            ROS_INFO_THROTTLE(2, "Current State: OVER (all tasks completed, stay in this state)");
            // 任务完成后可按需重置（如需循环执行，可在此处添加"重置为WAITDECODE"逻辑）
            break;

        default:
            ROS_WARN("Unknown State detected! Switch back to WAITDECODE");
            state_ = WAITDECODE;
            break;
        }

        // 处理ROS回调（必须调用，否则订阅话题无响应）
        ros::spinOnce();
        // 按10Hz频率休眠，平衡性能与响应速度
        rate.sleep();
    }

    // 程序退出时释放串口资源
    uart.close();
    ROS_INFO("UART port closed successfully, program exited normally");
    return 0;
}
