/*******************************************************
imgProc_softhand.h
    this is a revison from t42_img_control/imgProc.h
*******************************************************/
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// standard libraries
#include <iostream>

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
#include <image_transport/image_transport.h>

//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace cv;
using namespace std;

//-----------------------------------------------------
// CLASS
//-----------------------------------------------------
class imgProc
{
private:
    /*--- ros setup ---*/
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    /*--- image callback ---*/
    void imgCb(const sensor_msgs::ImageConstPtr& msg);

public:
    imgProc()
    :it(nh)
    {
        image_sub = it.subscribe("/pylon_camera_node/image_raw", 1, &imgProc::imgCb, this);
    }

    ~imgProc()
    {
    }

    /*--- images ---*/
    Mat img_src;     // subscribed 2D image
    Mat img_mask_obj; // object region
    Mat img_mask_hand;    // hand region
    Mat img_output;
    Mat img_black;  // for contact detection

    /*--- HSV parameter ---*/
    // parameter for robotic hand
    int H_MIN_HAND;
    int H_MAX_HAND;
    int S_MIN_HAND;
    int S_MAX_HAND;
    int V_MIN_HAND;
    int V_MAX_HAND;

    // parameter for object
    int H_MIN_OBJ;
    int H_MAX_OBJ;
    int S_MIN_OBJ;
    int S_MAX_OBJ;
    int V_MIN_OBJ;
    int V_MAX_OBJ;

    /*--- contours ---*/
    vector<vector<Point>> contours_obj;
    vector<vector<Point>> contours_hand;
    vector<vector<Point>> contours_hand2;
    vector<Point> contours_obj_endpoint;
    vector<Point> contours_hand_endpoint;
    vector<Point> contours_hand_deform;
    vector<Point> contours_hand_contact;


    /*--- centroid ---*/
    vector<Point> centroid_obj;
    vector<Point> centroid_hand;

    /*--- for contact detection using in drawContours ---*/
    int contact_hand_L = 0;
    int contact_hand_R = 0;

    /*--- string ---*/
    string mode_obj = "obj";
    string mode_hand = "hand";

    /*--- functions ---*/
    // extract region based on its color and create mask img
    Mat createMaskImg(Mat img, int H_MIN, int H_MAX, int S_MIN, int S_MAX, int V_MIN, int V_MAX);

    // calc contours
    vector<vector<Point>> getContours(Mat img_mask, string mode);

    // calc centroid
    vector<Point> calcCentroid(vector<vector<Point>> contours, string mode);

    // calc endpoint contour of obj
    vector<Point> findObjEndPoint(vector<vector<Point>> contours, vector<Point> centroid);

    // calc endpoint contour of hand
    vector<Point> findHandEndPoint(vector<vector<Point>> contours, vector<Point> centroid);

    // calc deformation
    vector<Point> calcDeformation(vector<Point> contours_endpoint);

    // drawContours
    void drawContours(vector<vector<Point>> contours, Mat img_mask, string mode);

    // determine target point
    Point determineTargetPoint(vector<Point> contours_hand_end, vector<Point> contours_deform, vector<Point> contours_obj_end);


    // get param
    bool getParam();

    bool exec();
};