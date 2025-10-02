// color_erea_detect把偏离位置发到 /red /blue /green
// /ZHU /ZHUI /GU
// state根据if判断选择订阅哪个并告诉下位机
// 还差串口部分
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <cmath>

enum State
{
    DECODE,
    WAIT2,
    PAI,
    WAIT3,
    DA,
    WAIT4,
    JIU,
    OVER
};

// 存储拆分后的数字
int data[3];

// 新增：存储各话题订阅到的float数据
float red_offset;   // /red话题数据（偏离值）
float green_offset; // /green话题数据（偏离值）
float blue_offset;  // /blue话题数据（偏离值）
float zhu_offset;   // /ZHU话题数据（偏离值）
float zhui_offset;  // /ZHUI话题数据（偏离值）
float gu_offset;    // /GU话题数据（偏离值）

#define t 0.1
#define y 0.1
#define u 0.1

// 新增：各话题的回调函数（接收float数据并存储）
void redCallback(const std_msgs::Float32::ConstPtr &msg)
{
    red_offset = msg->data;
    ROS_INFO("Received /red offset: %.2f", red_offset);
}

void greenCallback(const std_msgs::Float32::ConstPtr &msg)
{
    green_offset = msg->data;
    ROS_INFO("Received /green offset: %.2f", green_offset);
}

void blueCallback(const std_msgs::Float32::ConstPtr &msg)
{
    blue_offset = msg->data;
    ROS_INFO("Received /blue offset: %.2f", blue_offset);
}

void zhuCallback(const std_msgs::Float32::ConstPtr &msg)
{
    zhu_offset = msg->data;
    ROS_INFO("Received /ZHU offset: %.2f", zhu_offset);
}

void zhuiCallback(const std_msgs::Float32::ConstPtr &msg)
{
    zhui_offset = msg->data;
    ROS_INFO("Received /ZHUI offset: %.2f", zhui_offset);
}

void guCallback(const std_msgs::Float32::ConstPtr &msg)
{
    gu_offset = msg->data;
    ROS_INFO("Received /GU offset: %.2f", gu_offset);
}

// 新增：声明订阅器（需全局，避免作用域问题）
ros::Subscriber red_sub;
ros::Subscriber green_sub;
ros::Subscriber blue_sub;
ros::Subscriber zhu_sub;
ros::Subscriber zhui_sub;
ros::Subscriber gu_sub;

// 回调函数：处理接收到的3位整数
void decodeCallback(const std_msgs::Int32::ConstPtr &msg)
{
    int num = msg->data;

    // 验证是否为3位整数
    if (num >= 100 && num <= 999)
    {
        // 拆分数字：百位、十位、个位
        data[0] = num / 100;       // 百位数字
        data[1] = (num / 10) % 10; // 十位数字
        data[2] = num % 10;        // 个位数字

        // 打印结果以便验证
        ROS_INFO("Received number: %d", num);
        ROS_INFO("Split into array: [%d, %d, %d]", data[0], data[1], data[2]);
    }
    else
    {
        ROS_WARN("Received invalid number: %d (not a 3-digit number)", num);
    }
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "decode_processor");
    ros::NodeHandle nh;
    State state_ = DECODE;

    // 订阅/decode_info话题，设置回调函数
    ros::Subscriber sub = nh.subscribe("/decode_info", 10, decodeCallback);
    red_sub = nh.subscribe("/red", 10, redCallback);
    green_sub = nh.subscribe("/green", 10, greenCallback);
    blue_sub = nh.subscribe("/blue", 10, blueCallback);
    zhu_sub = nh.subscribe("/ZHU", 10, zhuCallback);
    zhui_sub = nh.subscribe("/ZHUI", 10, zhuiCallback);
    gu_sub = nh.subscribe("/GU", 10, guCallback);

    while (ros::ok())
    {
        switch (state_)
        {
        case DECODE:
            ROS_INFO_THROTTLE(1, "当前状态: 解码中"); // 每秒最多输出一次
            // 检查是否有有效解码数据
            if (data[0] != 0 || data[1] != 0 || data[2] != 0)
            {
                ROS_INFO("解码完成，发送指令A到下位机");
                // 向下位机发送指令"A"
                // 切换到等待状态，重置标志
                state_ = WAIT2;
            }
            break;

        case WAIT2:
            ROS_INFO_THROTTLE(1, "当前状态: 等待下位机回复2");
            if (1) // 收到下位机指令
            {
                ROS_INFO("收到下位机回复2，进入PAI状态");
                state_ = PAI;
            }
            break;

        case PAI:
            ROS_INFO("当前状态: PAI状态处理");
            if (data[0] == 1)
            {
                if (red_offset > t)
                {
                    // 串口发送 red_offset
                }
                else
                {
                    state_ = WAIT3;
                }
            }
            else if (data[0] == 2)
            {
                if (green_offset > t)
                {
                    // 串口发送 green_offset
                }
                else
                {
                    state_ = WAIT3;
                }
            }
            else // data[0] == 3
            {
                if (blue_offset > t)
                {
                    // 串口发送 blue_offset
                }
                else
                {
                    state_ = WAIT3;
                }
            }

            break;

        case WAIT3:
            ROS_INFO_THROTTLE(1, "当前状态: 等待下位机回复3");
            if (1) // 收到下位机指令
            {
                ROS_INFO("收到下位机回复3，进入DA状态");
                state_ = DA;
            }
            break;

        case DA:
            ROS_INFO("当前状态: DA状态处理");

            if (data[1] == 1)
            {
                if (red_offset > y)
                {
                    // 串口发送 red_offset
                }
                else
                {
                    state_ = WAIT4;
                }
            }
            else if (data[1] == 2)
            {
                if (green_offset > y)
                {
                    // 串口发送 green_offset
                }
                else
                {
                    state_ = WAIT4;
                }
            }
            else // data[1] == 3
            {
                if (blue_offset > y)
                {
                    // 串口发送 blue_offset
                }
                else
                {
                    state_ = WAIT4;
                }
            }
            break;

        case WAIT4:
            ROS_INFO_THROTTLE(1, "当前状态: 等待下位机回复3");
            if (1) // 收到下位机指令
            {
                ROS_INFO("收到下位机回复4，进入JIU状态");
                state_ = JIU;
            }
            break;

        case JIU:
            ROS_INFO("当前状态: JIU状态处理");

            if (data[2] == 1)
            {
                // 发送zhu_offset到串口
            }
            else if (data[2] == 2)
            {
                // 发送zhui_offset到串口
            }
            else // data[2] == 3
            {
                // 发送gu_offset到串口
            }
            if (1)
            {
                state_ = OVER; // 如果串口发送成功
            }
            else
            {
                continue;
            }
            break;

        default:
            ROS_WARN("未知状态，切换回解码状态"); // 关闭所有节点
            break;
        }

        ros::spinOnce(); // 新增：处理回调函数（避免订阅数据无法接收）
    }

    // 保持节点运行
    ros::spin();

    return 0;
}
