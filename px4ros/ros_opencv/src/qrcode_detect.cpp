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

void qrcode_identify(Mat &img);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection_node");
    ros::NodeHandle nh;

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

    Mat blur_img;
    GaussianBlur(gray_img, blur_img, Size(5, 5), 0);

    Mat binary_img;
    threshold(blur_img, binary_img, 0, 255, THRESH_BINARY | THRESH_OTSU);

    QRCodeDetector qrcode_detector;
    vector<Point> points;
    string decode_info = qrcode_detector.detectAndDecode(binary_img, points);

    if (!decode_info.empty())
    {
        cout << "Decoded QR Code:" << decode_info << endl;

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
        cout << "No QR Code" << endl;
    }
}