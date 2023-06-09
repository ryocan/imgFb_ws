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

  Mat src_color;
//-----------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------
void imgCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        src_color = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("src_color", src_color);
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

    // subscriber setting
    image_transport::Subscriber img_sub = it.subscribe("/pylon_camera_node/image_raw", 1, imgCb);

    ros::spin();

    return 0;
}