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

Mat img_src;
Mat img_black;
Mat img_output;

vector<vector<Point>> contours_obj;
vector<vector<Point>> contours_hand;
vector<Point> centroid_obj;

vector<Point> contours_match;
string mode_obj = "obj";
string mode_hand = "hand";
//-----------------------------------------------------
// FUNCTION
//-----------------------------------------------------
Mat extractRegion(Mat img, int H_MIN, int H_MAX, int S_MIN, int S_MAX, int V_MIN, int V_MAX)
{
    Mat img_mask = Mat::zeros(img.size(), img.type());

    // convert to HSV image
    Mat img_hsv;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    // extract region based on HSV parameter
    Scalar Lower(H_MIN, S_MIN, V_MIN);
    Scalar Upper(H_MAX, S_MAX, V_MAX);
    inRange(img_hsv, Lower, Upper, img_mask);

    return img_mask;
}

vector<vector<Point>> calcContours(Mat img_mask, string mode)
{
    // findContours from img_mask
    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    cv::findContours(img_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // detect based on area info
    double area = 0.;
    vector<vector<Point>> contours_output;

    // change contours calculation method
    if (mode == "obj")
    {
        // Assuming there is only one object, extract only the object with the largest area.
        double area_prev = 0.;
        for (int i = 0; i < contours.size(); i++)
        {
            area = contourArea(contours[i]);
            if (area > area_prev)
            {
                contours_output.clear();
                contours_output.shrink_to_fit();
                contours_output.push_back(contours[i]);
                area_prev = area;
            }
        }
    } 
    else if (mode == "hand")
    {
        // extract based on size of hand
        double area_th = 15000.;
        for (int i = 0; i < contours.size(); i++)
        {
            area = contourArea(contours[i]);
            if (area > area_th)
                contours_output.push_back(contours[i]);
        }
    }

    return contours_output;
}

vector<Point> calcCentroid(vector<vector<Point>> contours, string mode)
{
    vector<Point> centroid_output;

    // calc mu
    vector<Moments> mu;
    for (int i = 0; i < contours.size(); i++)
        mu.push_back(moments(contours[i], false));
    
    // display color setting
    Scalar color;
    if (mode == "obj")
        color = Scalar(0, 0, 255);
    else if (mode == "hand")
        color = Scalar(255, 0, 0);

    // calc mc    
    vector<Point2f> mc(contours.size());
    for (int i = 0; i < mu.size(); i++)
    {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

        if (isnan(mu[i].m10 / mu[i].m00) != true && isnan(mu[i].m01 / mu[i].m00) != true)
        {
            centroid_output.push_back(mc[i]);
            
            // display
            circle(img_output, mc[i], 10, Scalar(255, 255, 255), 8, 8);
            circle(img_output, mc[i],  8, color, 8, 8);
            circle(img_output, mc[i],  2, Scalar(255, 255, 255), 8, 8);
        }
    }

    return centroid_output;
}

void drawContours(vector<vector<Point>> contours, Mat img_mask, string mode)
{
    // declare for LineIterator
    Point li_start;
    Point li_goal;
    int line_width = 12;
    
    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours[i].size(); j++)
        {
            // specify start and end point for cv::LineIterator
            li_start = contours[i][j];

            if (j == contours[i].size() - 1)
                li_goal = contours[i][0];
            else    
                li_goal = contours[i][j + 1];

            // using cv::LineIterator
            LineIterator LI(img_mask, li_start, li_goal, 8, false);
        
            // get point on the line
            vector<Point> li_point(LI.count);
            for (int l = 0; l < LI.count; l++, ++LI)
                li_point[l] = LI.pos();

            // draw
            for (int k = 0; k < li_point.size(); k++)
            {
                // based on the object's centroid, change the direction to make the line thicker
                int sign_x = 1;
                if (li_point[k].x > centroid_obj[0].x) 
                    sign_x = -1;
                
                if (mode == "obj")
                    img_black.at<Vec3b>(li_point[k].y, li_point[k].x ) = Vec3b(0, 0, 255);
                else if (mode == "hand")
                    img_black.at<Vec3b>(li_point[k].y, li_point[k].x )[0] += 255; //overwrite on Hand's line


                if ((img_black.at<Vec3b>(li_point[k].y, li_point[k].x )) == Vec3b(255, 0, 255))
                {
                    contours_match.push_back(Point(li_point[k].x, li_point[k].y));
                }
            }
        }
    }
}

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char** argv)
{
    // node setup
    ros::init(argc, argv, "test0");

    img_src = imread("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/object2.jpg");
    img_black = Mat::zeros(img_src.size(), img_src.type());
    img_output = img_src.clone();
    imshow("img_src", img_src);

    // extract region
    Mat img_mask_obj = extractRegion(img_src, 101, 179, 241, 255, 0, 255);
    imshow("img_mask_obj", img_mask_obj);

    // calc contours
    contours_obj = calcContours(img_mask_obj, mode_obj);
    for (int i = 0; i < contours_obj.size(); i++)
        cv::drawContours(img_output, contours_obj, i, Scalar(0, 0, 255), 5);

    // calc centroid
    centroid_obj = calcCentroid(contours_obj, mode_obj);

    // draw contours
    drawContours(contours_obj, img_mask_obj, "obj");
    imshow("img_black", img_black);

    // add vertical line
    Mat img_black2 = Mat::zeros(img_src.size(), img_src.type());
    Point start(759, 1);
    Point end(759, 1023);
    line(img_black2, start, end, Scalar(0, 255, 0), 1);

    Mat img_mask_hand = extractRegion(img_black2, 5, 179, 0, 255, 0, 255);
    imshow("img_mask_hand", img_mask_hand);

    vector<Vec4i> hierarchy;
    cv::findContours(img_mask_hand, contours_hand, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    drawContours(contours_hand, img_mask_hand, mode_hand);
    imshow("img_black2", img_black);

    // contact position
    cout << contours_match << endl;
    for(int i = 0; i < contours_match.size(); i++)
    {
        circle(img_output, contours_match[i],  2, Scalar(255, 255, 255), 8, 8);
    } 

    // display
    imshow("img_output", img_output);

    int key = waitKey(0);
    // if (key == 'q')
    //     return -1;

    return 0;
}