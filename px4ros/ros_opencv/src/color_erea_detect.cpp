// color_erea_detect把偏离位置发到 /red /blue /green （颜色）
// /ZHU /ZHUI /GU  （形状）
// 中心坐标 320 240
// 还差调/red /blue /green阈值
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
// ROS相关头文件
#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace cv;

// 全局ROS节点句柄和发布器（方便各函数调用）
ros::NodeHandle *nh_ptr = nullptr;
ros::Publisher red_pub, green_pub, blue_pub; // 颜色话题发布器
ros::Publisher zhu_pub, zhui_pub, gu_pub;    // 灰色话题发布器

// 图像中心坐标（固定为320,240，与需求一致）
const Point IMG_CENTER(320, 240);

// 色块数据结构体：存储中心坐标、面积、颜色名称（用于排序和计算偏离）
struct BlockData
{
    Point center; // 色块中心坐标（原图坐标）
    double area;  // 色块面积
    string color; // 色块颜色

    // 排序规则：按面积降序（面积大的在前）
    bool operator<(const BlockData &other) const
    {
        return area > other.area;
    }
};

// 颜色检测参数结构体（HSV范围+颜色名称）
struct ColorDetectionParams
{
    Scalar lower;
    Scalar upper;
    string colorName;
};

// 计算偏离值（图像中心X - 色块中心X，返回float类型，若需Y方向可修改）
float calculateOffset(const Point &block_center)
{
    return static_cast<float>(IMG_CENTER.x - block_center.x);
}

// 发布颜色偏离值到对应话题（红色→/red，绿色→/green，蓝色→/blue）
void publishColorOffset(const string &color, float offset)
{
    std_msgs::Float32 msg;
    msg.data = offset;

    if (color == "红色")
    {
        red_pub.publish(msg);
        ROS_INFO("发布红色偏离值到/red：%.2f", offset);
    }
    else if (color == "绿色")
    {
        green_pub.publish(msg);
        ROS_INFO("发布绿色偏离值到/green：%.2f", offset);
    }
    else if (color == "蓝色")
    {
        blue_pub.publish(msg);
        ROS_INFO("发布蓝色偏离值到/blue：%.2f", offset);
    }
}

// 发布灰色色块偏离值（按面积排序：最大→/ZHU，第二→/ZHUI，第三→/GU）
void publishGrayOffsets(const vector<BlockData> &gray_blocks)
{
    std_msgs::Float32 msg;
    int block_count = gray_blocks.size();

    // 面积最大的灰色色块→/ZHU
    if (block_count >= 1)
    {
        msg.data = calculateOffset(gray_blocks[0].center);
        zhu_pub.publish(msg);
        ROS_INFO("发布最大灰色色块偏离值到/ZHU：%.2f（面积：%.0f像素）", msg.data, gray_blocks[0].area);
    }
    // 面积第二的灰色色块→/ZHUI
    if (block_count >= 2)
    {
        msg.data = calculateOffset(gray_blocks[1].center);
        zhui_pub.publish(msg);
        ROS_INFO("发布第二大灰色色块偏离值到/ZHUI：%.2f（面积：%.0f像素）", msg.data, gray_blocks[1].area);
    }
    // 面积第三的灰色色块→/GU
    if (block_count >= 3)
    {
        msg.data = calculateOffset(gray_blocks[2].center);
        gu_pub.publish(msg);
        ROS_INFO("发布第三大灰色色块偏离值到/GU：%.2f（面积：%.0f像素）", msg.data, gray_blocks[2].area);
    }
    // 若灰色色块不足3个，打印警告
    for (int i = block_count; i < 3; i++)
    {
        string topic = (i == 0) ? "/ZHU" : (i == 1) ? "/ZHUI"
                                                    : "/GU";
        ROS_WARN("灰色色块数量不足，未发布%s话题（当前仅检测到%d个有效灰色色块）", topic.c_str(), block_count);
    }
}

// 通用颜色检测函数：查找色块+标注+去色+偏离值发布
void detectColorBlock(const Mat &frame, const ColorDetectionParams &params, const Rect &roi_rect, Mat &output)
{
    Mat frame_roi = frame(roi_rect);
    Mat output_roi = output(roi_rect);
    Mat frame_HSV, color_mask;

    // 颜色空间转换与掩码生成
    cvtColor(frame_roi, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, params.lower, params.upper, color_mask);

    // 形态学优化（去噪+填补空洞）
    Mat kernel_small = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel_large = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(color_mask, color_mask, MORPH_OPEN, kernel_small);  // 开运算去噪
    morphologyEx(color_mask, color_mask, MORPH_CLOSE, kernel_large); // 闭运算填补空洞

    // 查找色块轮廓（仅保留外层轮廓）
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(color_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 存储当前颜色的有效色块
    vector<BlockData> color_blocks;
    for (size_t i = 0; i < contours.size(); i++)
    {
        // 过滤过小的轮廓（避免噪声干扰）
        double block_area = contourArea(contours[i]);
        if (block_area < 50)
            continue;

        // 计算色块中心点（转换为原图坐标，避免ROI偏移）
        Moments m = moments(contours[i]);
        if (m.m00 == 0) // 避免除零错误
            continue;
        int center_x = static_cast<int>(m.m10 / m.m00) + roi_rect.x;
        int center_y = static_cast<int>(m.m01 / m.m00) + roi_rect.y;
        Point block_center(center_x, center_y);

        // 存储色块数据
        color_blocks.push_back({block_center, block_area, params.colorName});

        // 图像标注（中心点+面积文本+轮廓）
        circle(output, block_center, 3, Scalar(0, 0, 255), -1); // 红色实心中心点
        string area_text = params.colorName + " Area: " + to_string(static_cast<int>(block_area));
        putText(output, area_text, Point(center_x, center_y + 15),
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);      // 黑色面积文本
        drawContours(output_roi, contours, i, Scalar(0, 255, 0), 1); // 绿色轮廓

        // 控制台输出色块信息
        cout << params.colorName << "色块" << i + 1 << "：面积=" << static_cast<int>(block_area)
             << "像素，中心=(" << center_x << "," << center_y << ")" << endl;
    }

    // 保留原功能：将检测到的色块区域设为黑色（去色）
    if (countNonZero(color_mask) > 100)
    {
        output_roi.setTo(Scalar(0, 0, 0), color_mask);
    }

    // 发布颜色偏离值（取面积最大的色块，若存在多个）
    if (!color_blocks.empty())
    {
        sort(color_blocks.begin(), color_blocks.end()); // 按面积降序排序
        float offset = calculateOffset(color_blocks[0].center);
        publishColorOffset(params.colorName, offset);
    }
    else
    {
        ROS_WARN("未检测到%s色块，未发布对应话题", params.colorName.c_str());
    }
}

// 红色检测（处理HSV双范围特性）+ 偏离值发布
void detectRedBlock(const Mat &frame, const Rect &roi_rect, Mat &output)
{
    // 红色HSV双范围（低H范围+高H范围）
    ColorDetectionParams redParams1 = {Scalar(0, 123, 192), Scalar(10, 249, 252), "红色"};
    ColorDetectionParams redParams2 = {Scalar(148, 100, 137), Scalar(168, 224, 238), "红色"};

    Mat frame_roi = frame(roi_rect);
    Mat output_roi = output(roi_rect);
    Mat frame_HSV, red_mask1, red_mask2, red_mask;

    // 生成双范围红色掩码并合并
    cvtColor(frame_roi, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, redParams1.lower, redParams1.upper, red_mask1);
    inRange(frame_HSV, redParams2.lower, redParams2.upper, red_mask2);
    bitwise_or(red_mask1, red_mask2, red_mask); // 合并两个红色掩码

    // 形态学优化（与通用颜色检测逻辑一致）
    Mat kernel_small = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel_large = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(red_mask, red_mask, MORPH_OPEN, kernel_small);
    morphologyEx(red_mask, red_mask, MORPH_CLOSE, kernel_large);

    // 查找红色色块轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(red_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 存储红色色块数据
    vector<BlockData> red_blocks;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double block_area = contourArea(contours[i]);
        if (block_area < 50)
            continue;

        Moments m = moments(contours[i]);
        if (m.m00 == 0)
            continue;
        int center_x = static_cast<int>(m.m10 / m.m00) + roi_rect.x;
        int center_y = static_cast<int>(m.m01 / m.m00) + roi_rect.y;
        Point block_center(center_x, center_y);

        // 存储红色色块数据
        red_blocks.push_back({block_center, block_area, "红色"});

        // 图像标注
        circle(output, block_center, 3, Scalar(0, 0, 255), -1);
        string area_text = "红色 Area: " + to_string(static_cast<int>(block_area));
        putText(output, area_text, Point(center_x, center_y + 15),
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
        drawContours(output_roi, contours, i, Scalar(0, 255, 0), 1);

        // 控制台输出
        cout << "红色色块" << i + 1 << "：面积=" << static_cast<int>(block_area)
             << "像素，中心=(" << center_x << "," << center_y << ")" << endl;
    }

    // 红色色块去色
    if (countNonZero(red_mask) > 100)
    {
        output_roi.setTo(Scalar(0, 0, 0), red_mask);
    }

    // 发布红色偏离值（取面积最大的红色色块）
    if (!red_blocks.empty())
    {
        sort(red_blocks.begin(), red_blocks.end());
        float offset = calculateOffset(red_blocks[0].center);
        publishColorOffset("红色", offset);
    }
    else
    {
        ROS_WARN("未检测到红色色块，未发布/red话题");
    }
}

// 灰色检测（按面积排序发布到/ZHU/ZHUI/GU）
void detectGrayBlocks(const Mat &frame, const Rect &roi_rect, Mat &output)
{
    ColorDetectionParams grayParams = {Scalar(0, 0, 156), Scalar(157, 63, 255), "灰色"};
    Mat frame_roi = frame(roi_rect);
    Mat output_roi = output(roi_rect);
    Mat frame_HSV, gray_mask;

    // 生成灰色掩码
    cvtColor(frame_roi, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, grayParams.lower, grayParams.upper, gray_mask);

    // 形态学优化
    Mat kernel_small = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel_large = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(gray_mask, gray_mask, MORPH_OPEN, kernel_small);
    morphologyEx(gray_mask, gray_mask, MORPH_CLOSE, kernel_large);

    // 查找灰色色块轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(gray_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 存储灰色色块数据
    vector<BlockData> gray_blocks;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double block_area = contourArea(contours[i]);
        if (block_area < 50)
            continue;

        Moments m = moments(contours[i]);
        if (m.m00 == 0)
            continue;
        int center_x = static_cast<int>(m.m10 / m.m00) + roi_rect.x;
        int center_y = static_cast<int>(m.m01 / m.m00) + roi_rect.y;
        Point block_center(center_x, center_y);

        // 存储灰色色块数据
        gray_blocks.push_back({block_center, block_area, "灰色"});

        // 图像标注
        circle(output, block_center, 3, Scalar(0, 0, 255), -1);
        string area_text = "灰色 Area: " + to_string(static_cast<int>(block_area));
        putText(output, area_text, Point(center_x, center_y + 15),
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
        drawContours(output_roi, contours, i, Scalar(0, 255, 0), 1);

        // 控制台输出
        cout << "灰色色块" << i + 1 << "：面积=" << static_cast<int>(block_area)
             << "像素，中心=(" << center_x << "," << center_y << ")" << endl;
    }

    // 按面积排序并发布灰色偏离值
    if (!gray_blocks.empty())
    {
        sort(gray_blocks.begin(), gray_blocks.end()); // 面积降序排序
        publishGrayOffsets(gray_blocks);
    }
    else
    {
        ROS_WARN("未检测到灰色色块，未发布/ZHU/ZHUI/GU话题");
    }
}

// 初始化ROS发布器（创建所有话题的发布器）
void initROSPublishers(ros::NodeHandle &nh)
{
    // 颜色话题：队列大小10（避免消息堆积）
    red_pub = nh.advertise<std_msgs::Float32>("/red", 10);
    green_pub = nh.advertise<std_msgs::Float32>("/green", 10);
    blue_pub = nh.advertise<std_msgs::Float32>("/blue", 10);

    // 灰色话题：队列大小10
    zhu_pub = nh.advertise<std_msgs::Float32>("/ZHU", 10);
    zhui_pub = nh.advertise<std_msgs::Float32>("/ZHUI", 10);
    gu_pub = nh.advertise<std_msgs::Float32>("/GU", 10);

    ROS_INFO("ROS发布器初始化完成！已创建话题：/red, /green, /blue, /ZHU, /ZHUI, /GU");
}

int main(int argc, char **argv)
{
    // 1. 初始化ROS节点
    ros::init(argc, argv, "color_gray_detection_node");
    ros::NodeHandle nh;
    nh_ptr = &nh;
    initROSPublishers(nh); // 初始化所有ROS发布器

    // 2. 初始化摄像头（设备号2）
    VideoCapture cap(2);
    if (!cap.isOpened())
    {
        ROS_ERROR("无法打开摄像头（设备号2）！请检查摄像头连接或设备号是否正确");
        cout << "无法打开摄像头" << endl;
        return -1;
    }
    ROS_INFO("摄像头初始化成功（设备号2），开始采集图像");

    // 3. 定义各颜色HSV参数（蓝色、绿色使用通用检测函数）
    ColorDetectionParams blueParams = {Scalar(87, 100, 141), Scalar(124, 225, 255), "蓝色"};
    ColorDetectionParams greenParams = {Scalar(35, 40, 40), Scalar(77, 255, 255), "绿色"}; // 标准绿色HSV范围

    Mat frame, output;
    ros::Rate rate(30); // ROS循环频率（30Hz，与摄像头帧率匹配）

    while (ros::ok())
    {
        // 4. 读取摄像头帧
        cap >> frame;
        if (frame.empty())
        {
            ROS_WARN("无法读取摄像头帧，跳过当前循环");
            cout << "无法读取帧" << endl;
            rate.sleep();
            continue;
        }
        output = frame.clone(); // 初始化输出图像（避免上一帧标注残留）

        // 5. 定义ROI区域（中心区域，占图像1/4大小）
        Rect roi_rect(
            frame.cols / 4, // ROI左上角X坐标
            frame.rows / 4, // ROI左上角Y坐标
            frame.cols / 2, // ROI宽度
            frame.rows / 2  // ROI高度
        );
        // 绘制ROI边框（绿色，线宽2，便于可视化ROI范围）
        rectangle(output, roi_rect, Scalar(0, 255, 0), 2);

        // 6. 颜色/灰色检测流程（按红→蓝→绿→灰顺序，避免颜色遮挡干扰）
        detectRedBlock(frame, roi_rect, output);                // 红色检测（双HSV范围）
        detectColorBlock(frame, blueParams, roi_rect, output);  // 蓝色检测（通用函数）
        detectColorBlock(frame, greenParams, roi_rect, output); // 绿色检测（通用函数）
        detectGrayBlocks(frame, roi_rect, output);              // 灰色检测（按面积排序发布）

        // 7. 显示检测结果窗口
        imshow("Color & Gray Block Detection", output);

        // 8. 按键退出逻辑（按'q'键关闭窗口并退出程序）
        if (waitKey(1) == 'q')
        {
            ROS_INFO("检测到'q'键按下，准备退出程序");
            break;
        }

        // 9. 保持ROS循环频率（避免CPU占用过高）
        ros::spinOnce(); // 处理ROS回调（此处无订阅回调，仅确保节点正常运行）
        rate.sleep();
    }

    // 10. 资源释放（关闭摄像头和窗口）
    cap.release();       // 释放摄像头资源
    destroyAllWindows(); // 关闭所有OpenCV窗口
    ROS_INFO("程序正常退出，已释放摄像头和窗口资源");

    return 0;
}
