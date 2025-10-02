#include <ros/ros.h>
#include <std_msgs/Int32.h> // 用于发布整数
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>
#include <string>
#include <sstream> // 用于字符串转换

using namespace cv;
using namespace std;

// 声明发布器为全局变量，以便在回调函数中使用
ros::Publisher decode_pub;

void qrcode_identify(Mat &img);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection_node");
    ros::NodeHandle nh;

    // 创建发布器，发布Int32类型到/decode_info话题
    decode_pub = nh.advertise<std_msgs::Int32>("/decode_info", 10);

    VideoCapture cap(2);
    if (!cap.isOpened())
    {
        cout << "Can not open the camera" << endl;
        return -1;
    }
    Mat frame;
    while (ros::ok())
    {
        cap >> frame;
        if (frame.empty())
        {
            cout << "Can not read the video" << endl;
            break;
        }
        qrcode_identify(frame);
        imshow("Frame", frame);

        int key = waitKey(1);
        if (key == 'q')
        {
            break;
        }
        ros::spinOnce();
    }
    cap.release();
    destroyAllWindows();

    return 0;
}

/*****封装检测二维码*****/
void qrcode_identify(Mat &img)
{
    Mat gray_img;
    cvtColor(img, gray_img, COLOR_BGR2GRAY);
    imshow("gray_img", gray_img);

    Mat binary_img;
    threshold(gray_img, binary_img, 0, 255, THRESH_BINARY | THRESH_OTSU);
    imshow("binary_img", binary_img);

    QRCodeDetector qrcode_detector;
    vector<Point> points;
    string decode_info = qrcode_detector.detectAndDecode(binary_img, points);

    if (!decode_info.empty())
    {
        cout << "Decoded QR Code:" << decode_info << endl;

        // 尝试将解码信息转换为整数
        try
        {
            // 转换为整数
            int code_num = stoi(decode_info);

            // 检查是否为3位数
            if (code_num >= 100 && code_num <= 999)
            {
                // 创建消息并发布
                std_msgs::Int32 msg;
                msg.data = code_num;
                decode_pub.publish(msg);
                ROS_INFO("Published QR code number: %d", code_num);
            }
            else
            {
                ROS_WARN("QR code is not a 3-digit number: %s", decode_info.c_str());
            }
        }
        catch (...)
        {
            ROS_ERROR("Failed to convert QR code content to integer: %s", decode_info.c_str());
        }

        if (!points.empty())
        {
            for (size_t i = 0; i < points.size(); i++)
            {
                line(img, points[i], points[(i + 1) % points.size()], Scalar(0, 255, 0), 2);
            }
        }
    }
    else
    {
        ROS_INFO("No QR Code detected");
    }
}
