/*******************************************************
pylon_image
*******************************************************/
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// about image processing
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// about ros
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <ros/package.h>

// standard libraries
#include <iostream>

//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;
using namespace cv;

Mat img_src, img_hsv, img_mask_obj, img_mask_hand;
int hmin_hand = 0, hmax_hand = 179, smin_hand = 0, smax_hand = 255, vmin_hand = 0, vmax_hand = 255;
int hmin_obj = 0, hmax_obj = 179, smin_obj = 0, smax_obj = 255, vmin_obj = 0, vmax_obj = 255;

//-----------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------
void imgCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_src = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // hsv 
    cvtColor(img_src, img_hsv, COLOR_BGR2HSV);
    cv::imshow("img_hsv", img_hsv);

    Scalar lower_obj(hmin_obj, smin_obj, vmin_obj);
    Scalar upper_obj(hmax_obj, smax_obj, vmax_obj);
    inRange(img_hsv, lower_obj, upper_obj, img_mask_obj);
    Mat element4 = (Mat_<uchar>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
    erode(img_mask_obj, img_mask_obj, element4, Point(-1, -1), 1);
    dilate(img_mask_obj, img_mask_obj, element4, Point(-1, -1), 1);

    Scalar lower_hand(hmin_hand, smin_hand, vmin_hand);
    Scalar upper_hand(hmax_hand, smax_hand, vmax_hand);
    inRange(img_hsv, lower_hand, upper_hand, img_mask_hand);

    // display
    cv::imshow("img_mask_obj", img_mask_obj);
    cv::imshow("img_mask_hand", img_mask_hand);

    waitKey(1);
}

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char** argv)
{
    // node setup
    ros::init(argc, argv, "pylon_img_show");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // hsv window
    namedWindow("Track Bar: HAND", (640, 200));
    createTrackbar("Hue min", "Track Bar: HAND", &hmin_hand, 179);
    createTrackbar("Hue max", "Track Bar: HAND", &hmax_hand, 179);
    createTrackbar("Sat min", "Track Bar: HAND", &smin_hand, 255);
    createTrackbar("Sat max", "Track Bar: HAND", &smax_hand, 255);
    createTrackbar("Val min", "Track Bar: HAND", &vmin_hand, 255);
    createTrackbar("Val max", "Track Bar: HAND", &vmax_hand, 255);
    namedWindow("Track Bar:OBJ", (640, 200));
    createTrackbar("Hue min", "Track Bar:OBJ", &hmin_obj, 179);
    createTrackbar("Hue max", "Track Bar:OBJ", &hmax_obj, 179);
    createTrackbar("Sat min", "Track Bar:OBJ", &smin_obj, 255);
    createTrackbar("Sat max", "Track Bar:OBJ", &smax_obj, 255);
    createTrackbar("Val min", "Track Bar:OBJ", &vmin_obj, 255);
    createTrackbar("Val max", "Track Bar:OBJ", &vmax_obj, 255);

    // subscriber setting
    image_transport::Subscriber img_sub = it.subscribe("/pylon_camera_node/image_raw", 1, imgCb);
    // image_transport::Subscriber img_sub = it.subscribe("/camera/color/image_raw", 1, imgCb);

    ros::spin();

    return 0;
}