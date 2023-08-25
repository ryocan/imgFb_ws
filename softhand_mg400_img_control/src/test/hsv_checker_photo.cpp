#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
    // ros setup
    ros::init(argc, argv, "hsv");

    int hmin = 0, hmax = 179, smin = 0, smax = 255, vmin = 0, vmax = 255;
    int bmin = 0, bmax = 255, gmin = 0, gmax = 255, rmin = 0, rmax = 255;

    Mat src = imread("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/input_images/with obj/input_1a.jpg");

    namedWindow("Track Bar", (640, 200));
    createTrackbar("Hue min", "Track Bar", &hmin, 179);
    createTrackbar("Hue max", "Track Bar", &hmax, 179);
    createTrackbar("Sat min", "Track Bar", &smin, 255);
    createTrackbar("Sat max", "Track Bar", &smax, 255);
    createTrackbar("Val min", "Track Bar", &vmin, 255);
    createTrackbar("Val max", "Track Bar", &vmax, 255);

    //createTrackbar("B min", "Track Bar", &bmin, 255);
    //createTrackbar("B max", "Track Bar", &bmax, 255);
    //createTrackbar("G min", "Track Bar", &gmin, 255);
    //createTrackbar("G max", "Track Bar", &gmax, 255);
    //createTrackbar("R min", "Track Bar", &rmin, 255);
    //createTrackbar("R max", "Track Bar", &rmax, 255);
    // HSVに変換

    Mat hsv;
    cvtColor(src, src, COLOR_BGR2HSV);
    int key = 0;

    while (true)
    {
        Mat mask;

        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        //Scalar lower(bmin, gmin, rmin);
        //Scalar upper(bmax, gmax, rmax);
        inRange(src, lower, upper, mask);
        
        imshow("img", src);
        imshow("mask", mask);
        
        cout << hmin << ", " << hmax << ", " << smin << ", " << smax << ", " << vmin << ", " << vmax << endl;
        
        int keyboard = waitKey(1);
        if (keyboard == 'q')
            break;
    }

    return 0;
}