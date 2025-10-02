#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

ros::Publisher detection_pub;

/*****检测色块*****/
struct ColorDetectionParams
{
    Scalar lower;
    Scalar upper;
    string colorName;
}; // 封装检测的颜色及其参数
ColorDetectionParams blueParams = {Scalar(100, 150, 0), Scalar(140, 255, 255), "blue"};
ColorDetectionParams redParams1 = {Scalar(0, 150, 0), Scalar(10, 255, 255), "red"};
ColorDetectionParams redParams2 = {Scalar(160, 150, 0), Scalar(180, 255, 255), "red"};

void detectColor(const Mat &frame, const ColorDetectionParams &params)
{
    if (params.colorName == "blue")
    {
        vector<Mat> bgr_planes;
        split(frame, bgr_planes);

        Mat blue_minus_red;
        subtract(bgr_planes[0], bgr_planes[2], blue_minus_red);

        Mat binary_blue_minus_red;
        threshold(blue_minus_red, binary_blue_minus_red, 30, 255, THRESH_BINARY);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(binary_blue_minus_red, binary_blue_minus_red, MORPH_OPEN, kernel);

        int pixel_count = countNonZero(binary_blue_minus_red);

        if (pixel_count > 6400)
        {
            cout << "detect" << params.colorName << "color" << endl;
            std_msgs::String msg;
            msg.data = "1";
            detection_pub.publish(msg);
        }
    }
    else
    {
        Mat frame_HSV;
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);

        Mat mask;
        inRange(frame_HSV, params.lower, params.upper, mask);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(mask, mask, MORPH_OPEN, kernel);

        int pixel_count = countNonZero(mask);

        if (pixel_count > 6400)
        {
            cout << "detect" << params.colorName << "color" << endl;
            std_msgs::String msg;
            msg.data = "2";
            detection_pub.publish(msg);
        }
    }
} // 颜色检测函数

void detectRed(const Mat &frame)
{
    Mat frame_HSV;
    cvtColor(frame, frame_HSV, COLOR_BGR2HSV);

    Mat red_mask1, red_mask2, red_mask;
    inRange(frame_HSV, redParams1.lower, redParams1.upper, red_mask1);
    inRange(frame_HSV, redParams2.lower, redParams2.upper, red_mask2);
    bitwise_or(red_mask1, red_mask2, red_mask);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(red_mask, red_mask, MORPH_OPEN, kernel);

    int red_pixel_count = countNonZero(red_mask);

    if (red_pixel_count > 100)
    {
        cout << "检测到红色色块" << endl;
        std_msgs::String msg;
        msg.data = "2";
        detection_pub.publish(msg);
    }
} // 检测红色

void color_identify(Mat &img)
{
    int imgWidth = img.cols;
    int imgHeight = img.rows;

    int roiWidth = 100;
    int roiHeight = 100;

    int roiX = (imgWidth - roiWidth) / 2;
    int roiY = (imgHeight - roiHeight) / 2;

    Rect roiRect(roiX, roiY, roiWidth, roiHeight);
    Mat roiImg = img(roiRect);

    rectangle(img, roiRect, Scalar(0, 0, 0), 2);

    detectColor(roiImg, blueParams);
    detectRed(roiImg);
} // 封装函数(使用roi确定检测区域)

/*****检测色球*****/
bool isColorPresent(const Mat &roi, const ColorDetectionParams &params)
{
    Mat hsv;
    cvtColor(roi, hsv, COLOR_BGR2HSV);
    Mat mask;
    inRange(hsv, params.lower, params.upper, mask);
    int nonZeroPixels = countNonZero(mask);
    double ratio = (double)nonZeroPixels / (roi.rows * roi.cols);
    return ratio > 0.6;
} // 检查指定 ROI 区域内是否存在指定颜色
bool isRedPresent(const Mat &roi)
{
    Mat hsv;
    cvtColor(roi, hsv, COLOR_BGR2HSV);
    Mat mask1, mask2;
    inRange(hsv, redParams1.lower, redParams1.upper, mask1);
    inRange(hsv, redParams2.lower, redParams2.upper, mask2);
    Mat mask;
    bitwise_or(mask1, mask2, mask);
    int nonZeroPixels = countNonZero(mask);
    double ratio = (double)nonZeroPixels / (roi.rows * roi.cols);
    return ratio > 0.6;
} // 检查指定 ROI 区域内是否存在红色

void detect_color_circle(Mat &image)
{
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);
    vector<Vec3f> circles;
    double dp = 2;
    double minDist = 10;
    double param1 = 100;
    double param2 = 100;
    int min_radius = 80;
    int max_radius = 120;
    HoughCircles(gray, circles, HOUGH_GRADIENT, dp, minDist, param1, param2, min_radius, max_radius);

    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        int x = center.x - radius;
        int y = center.y - radius;
        int width = 2 * radius;
        int height = 2 * radius;

        x = max(0, x);
        y = max(0, y);
        width = min(image.cols - x, width);
        height = min(image.rows - y, height);

        Rect roi_rect(x, y, width, height);
        Mat roi = image(roi_rect);

        string roi_window_name = "ROI " + to_string(i);
        imshow(roi_window_name, roi);

        Scalar circleColor;
        if (isColorPresent(roi, blueParams))
        {
            cout << "检测到" << blueParams.colorName << "球" << endl;
            std_msgs::String msg;
            msg.data = "1";
            detection_pub.publish(msg);
            circleColor = Scalar(0, 255, 0);
        }
        else if (isRedPresent(roi))
        {
            cout << "检测到红色球" << endl;
            std_msgs::String msg;
            msg.data = "2";
            detection_pub.publish(msg);
            circleColor = Scalar(255, 0, 255);
        }
        else
        {
            cout << "No color circle" << endl;
        }

        circle(image, center, 3, circleColor, -1, 8, 0);
        circle(image, center, radius, circleColor, 3, 8, 0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection_node");
    ros::NodeHandle nh;

    detection_pub = nh.advertise<std_msgs::String>("detection_topic", 10);

    VideoCapture cap(0);
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
        color_identify(frame);
        detect_color_circle(frame);
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
