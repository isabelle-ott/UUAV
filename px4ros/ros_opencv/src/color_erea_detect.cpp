// color_erea_detect把偏离位置发到 /red /blue /green /ZHU /ZHUI /GU /gray
// 中心坐标 320 240
// 还差调 red  blue  green阈值
// 缩小/gray 的roi区
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace cv;

// -------------------------- 新增：全局面积阈值（过滤小色块） --------------------------
const double mianji = 2000.0; // 面积阈值，单位：像素，过滤掉面积小于1000的色块
// -------------------------------------------------------------------------------------

// 全局ROS节点句柄和发布器（新增gray独立发布器，与原有灰色话题分离）
ros::NodeHandle *nh_ptr = nullptr;
ros::Publisher red_pub, green_pub, blue_pub; // 颜色话题发布器
ros::Publisher zhu_pub, zhui_pub, gu_pub;    // 原有灰色话题（/ZHU/ZHUI/GU）
ros::Publisher gray_indep_pub;               // 独立/gray话题发布器（与原有灰色话题完全分离）

// 图像中心坐标（固定为320,240，与需求一致）
const Point IMG_CENTER(320, 240);

// 色块数据结构体：存储中心坐标、面积、颜色名称
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

// 计算偏离值（图像中心X - 色块中心X，float类型）
float calculateOffset(const Point &block_center)
{
    return static_cast<float>(IMG_CENTER.x - block_center.x);
}

// 发布颜色偏离值到对应话题（红/绿/蓝）
void publishColorOffset(const string &color, float offset)
{
    std_msgs::Float32 msg;
    msg.data = offset;

    if (color == "红色")
    {
        red_pub.publish(msg);
        ROS_INFO("发布红色偏离值到/red：%.2f（面积≥%g像素）", offset, mianji);
    }
    else if (color == "绿色")
    {
        green_pub.publish(msg);
        ROS_INFO("发布绿色偏离值到/green：%.2f（面积≥%g像素）", offset, mianji);
    }
    else if (color == "蓝色")
    {
        blue_pub.publish(msg);
        ROS_INFO("发布蓝色偏离值到/blue：%.2f（面积≥%g像素）", offset, mianji);
    }
}

// 发布原有灰色话题（/ZHU/ZHUI/GU）：按面积排序，仅保留面积≥mianji的色块
void publishOriginalGrayOffsets(const vector<BlockData> &gray_blocks)
{
    std_msgs::Float32 msg;
    int block_count = gray_blocks.size();

    // 面积最大→/ZHU（已过滤，仅面积≥mianji的色块）
    if (block_count >= 1)
    {
        msg.data = calculateOffset(gray_blocks[0].center);
        zhu_pub.publish(msg);
        ROS_INFO("发布最大灰色色块偏离值到/ZHU：%.2f（面积：%.0f像素，≥%g像素）", msg.data, gray_blocks[0].area, mianji);
    }
    // 面积第二→/ZHUI（已过滤）
    if (block_count >= 2)
    {
        msg.data = calculateOffset(gray_blocks[1].center);
        zhui_pub.publish(msg);
        ROS_INFO("发布第二大灰色色块偏离值到/ZHUI：%.2f（面积：%.0f像素，≥%g像素）", msg.data, gray_blocks[1].area, mianji);
    }
    // 面积第三→/GU（已过滤）
    if (block_count >= 3)
    {
        msg.data = calculateOffset(gray_blocks[2].center);
        gu_pub.publish(msg);
        ROS_INFO("发布第三大灰色色块偏离值到/GU：%.2f（面积：%.0f像素，≥%g像素）", msg.data, gray_blocks[2].area, mianji);
    }
    // 不足3个时的警告（明确标注“面积≥%g像素”的条件）
    for (int i = block_count; i < 3; i++)
    {
        string topic = (i == 0) ? "/ZHU" : (i == 1) ? "/ZHUI"
                                                    : "/GU";
        ROS_WARN("灰色色块数量不足（需面积≥%g像素），未发布%s话题（当前仅检测到%d个有效灰色色块）", mianji, topic.c_str(), block_count);
    }
}

// 发布独立/gray话题：仅检测缩小后的ROI，且仅保留面积≥mianji的色块
void publishIndependentGrayOffset(const vector<BlockData> &gray_blocks)
{
    std_msgs::Float32 msg;
    if (!gray_blocks.empty())
    {
        // 取缩小ROI内面积最大的灰色色块（已过滤，仅面积≥mianji）
        BlockData largest_block = *max_element(gray_blocks.begin(), gray_blocks.end());
        msg.data = calculateOffset(largest_block.center);
        gray_indep_pub.publish(msg);
        ROS_INFO("发布缩小ROI内灰色色块偏离值到/gray：%.2f（面积：%.0f像素，≥%g像素）", msg.data, largest_block.area, mianji);
    }
    else
    {
        ROS_WARN("缩小ROI内未检测到有效灰色色块（需面积≥%g像素），未发布/gray话题", mianji);
    }
}

// 通用颜色检测函数（红/绿/蓝通用）：仅保留面积≥mianji的色块
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
    morphologyEx(color_mask, color_mask, MORPH_OPEN, kernel_small);
    morphologyEx(color_mask, color_mask, MORPH_CLOSE, kernel_large);

    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(color_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 存储有效色块（仅面积≥mianji的色块）
    vector<BlockData> color_blocks;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double block_area = contourArea(contours[i]);
        // -------------------------- 修改：过滤面积小于mianji的色块 --------------------------
        if (block_area < mianji)
        {
            ROS_DEBUG("%s色块面积%.0f像素（<%.0f像素），过滤掉", params.colorName.c_str(), block_area, mianji);
            continue;
        }
        // -------------------------------------------------------------------------------------

        Moments m = moments(contours[i]);
        if (m.m00 == 0)
            continue; // 避免除零
        int center_x = static_cast<int>(m.m10 / m.m00) + roi_rect.x;
        int center_y = static_cast<int>(m.m01 / m.m00) + roi_rect.y;
        Point block_center(center_x, center_y);

        // 存储色块数据
        color_blocks.push_back({block_center, block_area, params.colorName});

        // 图像标注（标注面积≥mianji的条件）
        circle(output, block_center, 3, Scalar(0, 0, 255), -1);
        string area_text = params.colorName + " Area: " + to_string(static_cast<int>(block_area)) + " (≥" + to_string(static_cast<int>(mianji)) + ")";
        putText(output, area_text, Point(center_x, center_y + 15),
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
        drawContours(output_roi, contours, i, Scalar(0, 255, 0), 1);

        // 控制台输出（明确标注有效色块）
        cout << params.colorName << "有效色块" << i + 1 << "：面积=" << static_cast<int>(block_area)
             << "像素（≥" << static_cast<int>(mianji) << "），中心=(" << center_x << "," << center_y << ")" << endl;
    }

    // 检测区域去色（仅对有效色块区域去色）
    if (countNonZero(color_mask) > mianji / 2) // 去色阈值关联面积阈值，避免小区域误去色
    {
        output_roi.setTo(Scalar(0, 0, 0), color_mask);
    }

    // 发布偏离值（仅对有效色块发布）
    if (!color_blocks.empty())
    {
        sort(color_blocks.begin(), color_blocks.end());
        float offset = calculateOffset(color_blocks[0].center);
        publishColorOffset(params.colorName, offset);
    }
    else
    {
        ROS_WARN("未检测到%s有效色块（需面积≥%g像素），未发布对应话题", params.colorName.c_str(), mianji);
    }
}

// 红色检测（双HSV范围）：仅保留面积≥mianji的色块
void detectRedBlock(const Mat &frame, const Rect &roi_rect, Mat &output)
{
    ColorDetectionParams redParams1 = {Scalar(0, 100, 100), Scalar(10, 255, 255), "红色"};
    ColorDetectionParams redParams2 = {Scalar(160, 100, 100), Scalar(179, 255, 255), "红色"};

    Mat frame_roi = frame(roi_rect);
    Mat output_roi = output(roi_rect);
    Mat frame_HSV, red_mask1, red_mask2, red_mask;

    // 双范围掩码合并
    cvtColor(frame_roi, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, redParams1.lower, redParams1.upper, red_mask1);
    inRange(frame_HSV, redParams2.lower, redParams2.upper, red_mask2);
    bitwise_or(red_mask1, red_mask2, red_mask);

    // 形态学优化
    Mat kernel_small = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel_large = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(red_mask, red_mask, MORPH_OPEN, kernel_small);
    morphologyEx(red_mask, red_mask, MORPH_CLOSE, kernel_large);

    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(red_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 存储红色有效色块（仅面积≥mianji）
    vector<BlockData> red_blocks;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double block_area = contourArea(contours[i]);
        // -------------------------- 修改：过滤面积小于mianji的色块 --------------------------
        if (block_area < mianji)
        {
            ROS_DEBUG("红色色块面积%.0f像素（<%.0f像素），过滤掉", block_area, mianji);
            continue;
        }
        // -------------------------------------------------------------------------------------

        Moments m = moments(contours[i]);
        if (m.m00 == 0)
            continue;
        int center_x = static_cast<int>(m.m10 / m.m00) + roi_rect.x;
        int center_y = static_cast<int>(m.m01 / m.m00) + roi_rect.y;
        Point block_center(center_x, center_y);

        red_blocks.push_back({block_center, block_area, "红色"});

        // 图像标注（标注面积≥mianji的条件）
        circle(output, block_center, 3, Scalar(0, 0, 255), -1);
        string area_text = "红色 Area: " + to_string(static_cast<int>(block_area)) + " (≥" + to_string(static_cast<int>(mianji)) + ")";
        putText(output, area_text, Point(center_x, center_y + 15),
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
        drawContours(output_roi, contours, i, Scalar(0, 255, 0), 1);

        // 控制台输出（明确标注有效色块）
        cout << "红色有效色块" << i + 1 << "：面积=" << static_cast<int>(block_area)
             << "像素（≥" << static_cast<int>(mianji) << "），中心=(" << center_x << "," << center_y << ")" << endl;
    }

    // 红色区域去色（仅对有效色块区域去色）
    if (countNonZero(red_mask) > mianji / 2)
    {
        output_roi.setTo(Scalar(0, 0, 0), red_mask);
    }

    // 发布红色偏离值（仅对有效色块发布）
    if (!red_blocks.empty())
    {
        sort(red_blocks.begin(), red_blocks.end());
        float offset = calculateOffset(red_blocks[0].center);
        publishColorOffset("红色", offset);
    }
    else
    {
        ROS_WARN("未检测到红色有效色块（需面积≥%g像素），未发布/red话题", mianji);
    }
}

// 原有灰色检测（对应/ZHU/ZHUI/GU）：仅保留面积≥mianji的色块
void detectOriginalGrayBlocks(const Mat &frame, const Rect &original_roi, Mat &output)
{
    ColorDetectionParams grayParams = {Scalar(0, 0, 153), Scalar(109, 67, 255), "灰色"};
    Mat frame_roi = frame(original_roi);
    Mat output_roi = output(original_roi);
    Mat frame_HSV, gray_mask;

    // 掩码生成与优化
    cvtColor(frame_roi, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, grayParams.lower, grayParams.upper, gray_mask);
    Mat kernel_small = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel_large = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(gray_mask, gray_mask, MORPH_OPEN, kernel_small);
    morphologyEx(gray_mask, gray_mask, MORPH_CLOSE, kernel_large);

    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(gray_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 存储灰色有效色块（仅面积≥mianji）
    vector<BlockData> gray_blocks;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double block_area = contourArea(contours[i]);
        // -------------------------- 修改：过滤面积小于mianji的色块 --------------------------
        if (block_area < mianji)
        {
            ROS_DEBUG("原灰色色块面积%.0f像素（<%.0f像素），过滤掉", block_area, mianji);
            continue;
        }
        // -------------------------------------------------------------------------------------

        Moments m = moments(contours[i]);
        if (m.m00 == 0)
            continue;
        int center_x = static_cast<int>(m.m10 / m.m00) + original_roi.x;
        int center_y = static_cast<int>(m.m01 / m.m00) + original_roi.y;
        Point block_center(center_x, center_y);

        gray_blocks.push_back({block_center, block_area, "灰色"});

        // 图像标注（标注面积≥mianji的条件，与原灰色区分）
        circle(output, block_center, 3, Scalar(0, 255, 0), -1);
        string area_text = "原灰色 Area: " + to_string(static_cast<int>(block_area)) + " (≥" + to_string(static_cast<int>(mianji)) + ")";
        putText(output, area_text, Point(center_x, center_y + 15),
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 0, 0), 1);    // 蓝色文本区分
        drawContours(output_roi, contours, i, Scalar(255, 0, 0), 1); // 蓝色轮廓区分

        // 控制台输出有效灰色色块信息
        cout << "原灰色有效色块" << i + 1 << "：面积=" << static_cast<int>(block_area)
             << "像素（≥" << static_cast<int>(mianji) << "），中心=(" << center_x << "," << center_y << ")" << endl;
    }

    // 发布原灰色话题（/ZHU/ZHUI/GU）：仅基于有效色块（面积≥mianji）
    if (!gray_blocks.empty())
    {
        sort(gray_blocks.begin(), gray_blocks.end());
        publishOriginalGrayOffsets(gray_blocks);
    }
    else
    {
        ROS_WARN("原ROI内未检测到灰色有效色块（需面积≥%g像素），未发布/ZHU/ZHUI/GU话题", mianji);
    }
}

// 独立灰色检测（对应/gray）：缩小ROI+仅保留面积≥mianji的色块
void detectIndependentGrayBlock(const Mat &frame, const Rect &small_roi, Mat &output)
{
    ColorDetectionParams grayParams = {Scalar(0, 0, 153), Scalar(109, 67, 255), "灰色"};
    Mat frame_roi = frame(small_roi);
    Mat output_roi = output(small_roi);
    Mat frame_HSV, gray_mask;

    // 掩码生成与优化（与原灰色参数一致，仅ROI不同）
    cvtColor(frame_roi, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, grayParams.lower, grayParams.upper, gray_mask);
    Mat kernel_small = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel_large = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(gray_mask, gray_mask, MORPH_OPEN, kernel_small);
    morphologyEx(gray_mask, gray_mask, MORPH_CLOSE, kernel_large);

    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(gray_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 存储缩小ROI内的灰色有效色块（仅面积≥mianji）
    vector<BlockData> small_gray_blocks;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double block_area = contourArea(contours[i]);
        // 过滤面积小于mianji的色块
        if (block_area < mianji)
        {
            ROS_DEBUG("缩小ROI灰色色块面积%.0f像素（<%.0f像素），过滤掉", block_area, mianji);
            continue;
        }

        Moments m = moments(contours[i]);
        if (m.m00 == 0)
            continue;
        int center_x = static_cast<int>(m.m10 / m.m00) + small_roi.x;
        int center_y = static_cast<int>(m.m01 / m.m00) + small_roi.y;
        Point block_center(center_x, center_y);

        small_gray_blocks.push_back({block_center, block_area, "灰色"});

        // 图像标注（标注面积≥mianji的条件，与原灰色区分）
        circle(output, block_center, 3, Scalar(0, 255, 0), -1);
        string area_text = "缩小ROI灰色 Area: " + to_string(static_cast<int>(block_area)) + " (≥" + to_string(static_cast<int>(mianji)) + ")";
        putText(output, area_text, Point(center_x, center_y + 30),   // 文本位置下移避免重叠
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 255), 1);  // 黄色文本区分
        drawContours(output_roi, contours, i, Scalar(0, 255, 0), 1); // 黄色轮廓区分

        // 控制台输出缩小ROI内有效灰色色块信息
        cout << "缩小ROI灰色有效色块" << i + 1 << "：面积=" << static_cast<int>(block_area)
             << "像素（≥" << static_cast<int>(mianji) << "），中心=(" << center_x << "," << center_y << ")" << endl;
    }

    // 发布独立/gray话题（基于缩小ROI内的有效色块）
    publishIndependentGrayOffset(small_gray_blocks);
}

// 初始化ROS发布器（明确区分所有话题）
void initROSPublishers(ros::NodeHandle &nh)
{
    // 颜色话题
    red_pub = nh.advertise<std_msgs::Float32>("/red", 10);
    green_pub = nh.advertise<std_msgs::Float32>("/green", 10);
    blue_pub = nh.advertise<std_msgs::Float32>("/blue", 10);

    // 原灰色话题（/ZHU/ZHUI/GU）
    zhu_pub = nh.advertise<std_msgs::Float32>("/ZHU", 10);
    zhui_pub = nh.advertise<std_msgs::Float32>("/ZHUI", 10);
    gu_pub = nh.advertise<std_msgs::Float32>("/GU", 10);

    // 独立/gray话题（与原灰色话题完全分离）
    gray_indep_pub = nh.advertise<std_msgs::Float32>("/gray", 10);

    ROS_INFO("ROS发布器初始化完成！已创建话题：/red, /green, /blue, /ZHU, /ZHUI, /GU, /gray | 色块面积过滤阈值：%g像素", mianji);
}

int main(int argc, char **argv)
{
    // 1. 初始化ROS节点
    ros::init(argc, argv, "color_gray_detection_node");
    ros::NodeHandle nh;
    nh_ptr = &nh;
    initROSPublishers(nh);

    // 2. 初始化摄像头（设备号2）
    VideoCapture cap(2);
    if (!cap.isOpened())
    {
        ROS_ERROR("无法打开摄像头（设备号2）！请检查连接");
        cout << "无法打开摄像头" << endl;
        return -1;
    }
    ROS_INFO("摄像头初始化成功（设备号2）");

    // 3. 定义颜色HSV参数（优化后阈值，适配面积过滤）
    ColorDetectionParams redParams1 = {Scalar(0, 120, 80), Scalar(10, 255, 255), "红色"};    // 优化红色低范围（减少误检）
    ColorDetectionParams redParams2 = {Scalar(160, 120, 80), Scalar(179, 255, 255), "红色"}; // 优化红色高范围
    ColorDetectionParams blueParams = {Scalar(95, 150, 50), Scalar(125, 255, 255), "蓝色"};  // 优化蓝色阈值（增强抗干扰）
    ColorDetectionParams greenParams = {Scalar(35, 80, 80), Scalar(77, 255, 255), "绿色"};   // 优化绿色阈值（减少环境光影响）

    Mat frame, output;
    ros::Rate rate(30);

    while (ros::ok())
    {
        // 4. 读取摄像头帧
        cap >> frame;
        if (frame.empty())
        {
            ROS_WARN("无法读取摄像头帧，跳过当前循环");
            rate.sleep();
            continue;
        }
        output = frame.clone();

        // 5. 定义两个ROI区域
        // 5.1 原灰色ROI（中心1/4区域，用于/ZHU/ZHUI/GU）
        Rect original_gray_roi(
            frame.cols / 4,
            frame.rows / 4,
            frame.cols / 2,
            frame.rows / 2);
        rectangle(output, original_gray_roi, Scalar(255, 0, 0), 2); // 蓝色边框标记（标注ROI类型）
        putText(output, "Original ROI (/ZHU/ZHUI/GU)", Point(original_gray_roi.x, original_gray_roi.y - 5),
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 0, 0), 1);

        // 5.2 缩小ROI（中心1/8区域，用于/gray）
        Rect small_gray_roi(
            frame.cols * 3 / 8, // X起点：图像宽度3/8处（中心对齐）
            frame.rows * 3 / 8, // Y起点：图像高度3/8处（中心对齐）
            frame.cols / 4,     // 宽度：原ROI的1/2（缩小后更聚焦中心）
            frame.rows / 4      // 高度：原ROI的1/2
        );
        rectangle(output, small_gray_roi, Scalar(0, 255, 255), 2); // 黄色边框标记
        putText(output, "Small ROI (/gray)", Point(small_gray_roi.x, small_gray_roi.y - 5),
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 255), 1);

        // 6. 检测流程（按红→蓝→绿→原灰色→缩小ROI灰色顺序，避免遮挡干扰）
        detectRedBlock(frame, original_gray_roi, output);                // 红色检测（用原ROI）
        detectColorBlock(frame, blueParams, original_gray_roi, output);  // 蓝色检测（用原ROI）
        detectColorBlock(frame, greenParams, original_gray_roi, output); // 绿色检测（用原ROI）
        detectOriginalGrayBlocks(frame, original_gray_roi, output);      // 原灰色检测（/ZHU/ZHUI/GU）
        detectIndependentGrayBlock(frame, small_gray_roi, output);       // 缩小ROI灰色检测（/gray）

        // 7. 显示结果（标注面积阈值信息）
        putText(output, "Area Threshold: " + to_string(static_cast<int>(mianji)) + " Pixels", Point(10, 20),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1); // 红色文本标注面积阈值
        imshow("Color & Gray Detection (Dual ROI + Area Filter)", output);

        // 8. 退出逻辑
        if (waitKey(1) == 'q')
        {
            ROS_INFO("检测到'q'键，退出程序");
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 9. 资源释放
    cap.release();
    destroyAllWindows();
    ROS_INFO("程序正常退出");

    return 0;
}
