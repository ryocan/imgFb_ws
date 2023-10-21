// 手法改善(接触判定，長さ調整)
// 70のとき

//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// standard libraries
#include <iostream>
#include <math.h>

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

// csv
#include <fstream>

// namespace
using namespace cv;
using namespace std;
//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
// path
int segment_N = 50;
string input_img_num = "/115_5";
string input_obj = "/setup";
string input_path = "/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/input_images/robosym/new2" + input_obj;
string output_path = "/home/umelab/imgFb_ws/src/softhand_mg400_img_control/result/new2" + input_obj + input_img_num + "/";

//  Image
Mat img_input_normal;
Mat img_input_curve;
Mat img_mask_hand;
Mat img_output_contact;
Mat img_output_est;
Mat img_output_curve;

// System
int Flag = 0;

// Calib
double theta_setup = -0.426985;
// double length_setup = 233.;
double length_setup = 0.;
double theta_segment_setup = 0.;
double length_segment = 0.;

// Point
Point point_hand_bottom;
Point point_hand_top;
Point point_contact_1;
Point point_contact_2;

// Curve estimation
Point point_standard;

// contact 1
double length_contact_1 = 0.;
double theta_contact_1 = 0.;
int segment_contact_1 = 0;
double theta_segment_contact_1 = 0.;

// contact 2
Mat img_black;
Mat img_black_circle;
Mat img_mask_obj;
Mat img_mask_black_circle;
vector<Point> contours_obj;
vector<Point> contours_circle;
double theta_contact_2 = 0.;
int segment_contact_2 = 0;
double theta_segment_contact_2 = 0.;
double theta_diff = 0.;
double theta_segment_remain = 0.;

// Result
vector<Point> point_result_estimation;
vector<Point> point_result_true;

// // Valid
// Mat img_input_valid;
// Mat img_mask_valid;

//-----------------------------------------------------
// FUNCTIONS: General
//-----------------------------------------------------
Mat createMaskImg(Mat img, int H_MIN, int H_MAX, int S_MIN, int S_MAX, int V_MIN, int V_MAX)
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

vector<Point> getContours(Mat img_mask)
{
    // findContours from img_mask
    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    // cv::findContours(img_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    cv::findContours(img_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);    //8近傍mode

    // detect based on area info
    double area = 0.;
    vector<Point> contours_output;

    // change contours calculation method
    double area_prev = 0.;
    for (int i = 0; i < contours.size(); i++)
    {
        area = contourArea(contours[i]);
        if (area > area_prev && area > 50)
        {
            contours_output.clear();
            contours_output = contours[i];
            area_prev = area;
        }
    }

    return contours_output;
}

void curveEstimation(int segment_start, int segment_end, double theta_start, double theta_segment)
{
    double theta_accumulate = 0;
    double length_accumulate = 0.;

    cout << "----- < Curve Estimation > -----" << endl;
    cout << "theta_segment: " << theta_segment << endl;

    for (int i = segment_start; i < segment_end; i++)  
    {
        cout << "i: " << i << endl;
        // bending angle of each segment
        theta_accumulate = theta_start + theta_segment * (2 * i + 1);
        cout << "theta_accumulate: " << theta_accumulate << endl;
        
        // calculated slope of the line from bending angle
        double slope = tan(3.14/2 - theta_accumulate);
        cout << "slope: " << slope << endl; 

        cout << "point_standard: " << point_standard << endl; 

        // estimate curve end point
        double j = 0.1;
        while(true)
        {
            double y = point_standard.y + j;
            double x = (y - point_standard.y) / slope + point_standard.x;

            double diff_length = sqrt(pow(x - point_standard.x, 2) + pow(y - point_standard.y, 2));
            
            if( diff_length > length_segment)
            {
                point_result_estimation.push_back(Point(x, y));
                point_standard = Point(x, y);
                cout << "-----< " << diff_length << " >-----" << endl;
                break;
            }
            j += 0.1;
        }
    }
}

void drawMyContours(vector<Point> contours, Mat img_mask, string mode, Mat img_black)
{
    // declare for LineIterator
    Point li_start;
    Point li_goal;
    int line_width = 1;

    Point point_output = Point(0,0);
    
    for (int i = 0; i < contours.size(); i++)
    {
        // specify start and end point for cv::LineIterator
        li_start = contours[i];

        if (i == contours.size() - 1)
            li_goal = contours[0];
        else    
            li_goal = contours[i + 1];

        // using cv::LineIterator
        LineIterator LI(img_mask, li_start, li_goal, 8, false);
    
        // get point on the line
        vector<Point> li_point(LI.count);
        for (int l = 0; l < LI.count; l++, ++LI)
            li_point[l] = LI.pos();

        // draw
        for (int k = 0; k < li_point.size(); k++)
        {
            if (mode == "obj")
            {
                img_black.at<Vec3b>(li_point[k].y, li_point[k].x) = Vec3b(0, 0, 255);
                img_black.at<Vec3b>(li_point[k].y + 1, li_point[k].x) = Vec3b(0, 0, 255);
                img_black.at<Vec3b>(li_point[k].y - 1, li_point[k].x) = Vec3b(0, 0, 255);
            }
            else if (mode == "hand")
            {
                img_black.at<Vec3b>(li_point[k].y, li_point[k].x)[0] += 255; //overwrite on Hand's line
                img_black.at<Vec3b>(li_point[k].y + 1, li_point[k].x)[0] += 255; //overwrite on Hand's line
                img_black.at<Vec3b>(li_point[k].y - 1, li_point[k].x)[0] += 255; //overwrite on Hand's line
            }



            if ((img_black.at<Vec3b>(li_point[k].y, li_point[k].x)) == Vec3b(255, 0, 255))
            {
                cout << "contact pos: " << "( " << li_point[k].x << ", " << li_point[k].y << " )" << endl;
                if( li_point[k].y > point_output.y)
                    point_output = li_point[k];
            }
            
        }
    }

    if (mode == "hand")
        point_contact_2 = point_output;
    
}

//-----------------------------------------------------
// FUNCTIONS: Process
//-----------------------------------------------------
void calibResult()
{
    cout << "----- < Calib Result > ----- " << endl;
    length_setup = sqrt(pow(point_hand_top.x - point_hand_bottom.x, 2) + pow(point_hand_top.y - point_hand_bottom.y, 2));

    theta_segment_setup = theta_setup / segment_N;
    length_segment = length_setup / segment_N;

    cout << "theta_setup: " << theta_setup << endl;
    cout << "length_setup: " << length_setup << endl;
    cout << "length_segment: " << length_segment << endl;
}

void handExtraction()
{
    // create mask img
    img_mask_hand = createMaskImg(img_input_normal, 0, 179, 0, 255, 0, 70);
    imshow("img_mask_hand", img_mask_hand);

    // findContours from img_mask
    vector<Point> contours_hand;    
    contours_hand = getContours(img_mask_hand);

    // find endpoint candidate
    int Phb_N = 0;  // num of P_b(Point bottom)
    int Pht_N = 0;  // num of P_t(Point top)
    int Pht_y_prev = 0;
    int Phb_y_prev = 320;

    int Phtl_N = 0;  // endpoint
    int Phbl_N = 0;
    int Phtl_x_prev = 640;
    int Phbl_x_prev = 640;

    // Find endpoint: First time scan
    for (int i = 0; i < contours_hand.size(); i++)
    {
        if(contours_hand[i].y < Phb_y_prev)
        {
            Phb_y_prev = contours_hand[i].y;
            Phb_N = i;
        }
        if(contours_hand[i].y > Pht_y_prev)
        {
            Pht_y_prev = contours_hand[i].y;
            Pht_N = i;
        }
    }

    // Find endpoint: Second time scan
    for (int i = 0; i < contours_hand.size(); i++)
    {
        if(contours_hand[i].y < contours_hand[Phb_N].y + 10)
        {
            if(contours_hand[i].x < Phbl_x_prev)
            {
                Phbl_x_prev = contours_hand[i].x;
                Phbl_N = i;
            }
        }
        if(contours_hand[Pht_N].y - 10 < contours_hand[i].y )
        {
            if(contours_hand[i].x < Phtl_x_prev)
            {
                Phtl_x_prev = contours_hand[i].x;
                Phtl_N = i;
            }
        }
    }

    // Substitution
    point_hand_bottom = contours_hand[Phbl_N];
    point_hand_top = contours_hand[Phtl_N];

    // Show
    circle(img_output_contact, point_hand_bottom,  8, Scalar(255,   255,   255), -1);
    circle(img_output_contact, point_hand_bottom,  6, Scalar(255,   0,   0), -1);
    circle(img_output_contact, point_hand_top,  8, Scalar(255,   255,   255), -1);
    circle(img_output_contact, point_hand_top,  6, Scalar(255,   0,   0), -1);
    imshow("img_output_contact", img_output_contact);
}

void objExtractionRoi()
{
    // convert to mask img
    Mat img_input_normal_roi(img_input_normal, cv::Rect(0, 0, point_hand_top.x + 20, point_hand_top.y));
    Mat img_mask_obj_roi = createMaskImg(img_input_normal_roi, 0, 15, 0, 255, 80, 255);
    imshow("img_mask_obj_roi", img_mask_obj_roi);
    imwrite(output_path + input_img_num + "_" + to_string(segment_N) + "_mask_obj_roi"  + ".jpg", img_mask_obj_roi);

    // get contour
    vector<Point> contours_obj_roi;    
    contours_obj_roi = getContours(img_mask_obj_roi);
    if(contours_obj_roi.empty())
    {
        point_contact_1 = Point(0, 0);
    }
    else
    {
        // find right/left endpoint
        int Pobj_N = 0;  // endpoint of the object in the right side
        int Pobj_x_prev = 0;

        for (int i = 0; i < contours_obj_roi.size(); i++)
        {
            if(contours_obj_roi[i].x > Pobj_x_prev )
            {
                Pobj_x_prev = contours_obj_roi[i].x;
                Pobj_N = i;
            }
        }

        // Substitution
        point_contact_1 = contours_obj_roi[Pobj_N];

        // Show
        circle(img_output_contact, point_contact_1,  8, Scalar(255,   255,   255), -1);
        circle(img_output_contact, point_contact_1,  6, Scalar(  0,   0,     255), -1);
        imshow("img_output_contact", img_output_contact);
    }

}

void objContour()
{
    img_mask_obj = createMaskImg(img_input_normal, 0, 15, 0, 255, 80, 255);
    imshow("img_mask_obj", img_mask_obj);
 
    contours_obj = getContours(img_mask_obj);
}


void circleForDetectContact()
{
    double radius = length_setup - length_contact_1;
    Point circle_center = point_contact_1;

    cv::circle(img_black_circle, circle_center, radius, cv::Scalar(0,255,0), 1, cv::LINE_4);
    cv::circle(img_output_contact, circle_center, radius, cv::Scalar(0,255,0), 1, cv::LINE_4);
    // imshow("img_black_circle", img_black_circle);
    imshow("img_output_contact", img_output_contact);

    img_mask_black_circle = createMaskImg(img_black_circle, 10, 179, 0, 255, 0, 255);
    contours_circle = getContours(img_mask_black_circle);
}

void validation(vector<Point> contours, Mat img_mask, string mode, Mat img_black)
{
    // declare for LineIterator
    Point li_start;
    Point li_goal;
    int line_width = 1;

    Point point_output = Point(0,0);
    
    for (int i = 0; i < contours.size(); i++)
    {
        // specify start and end point for cv::LineIterator
        li_start = contours[i];

        if (i == contours.size() - 1)
            li_goal = contours[0];
        else    
            li_goal = contours[i + 1];

        // using cv::LineIterator
        LineIterator LI(img_mask, li_start, li_goal, 8, false);
    
        // get point on the line
        vector<Point> li_point(LI.count);
        for (int l = 0; l < LI.count; l++, ++LI)
            li_point[l] = LI.pos();

        
        for (int k = 0; k < li_point.size(); k+=10)
        {
            img_black.at<Vec3b>(li_point[k].y, li_point[k].x) = Vec3b(0, 0, 255);
            
        }
    }
    
}
//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    // ------------------------- < SETUP > ----------------------------
    // ros setup
    ros::init(argc, argv, "simulate_with_obj");

    // input img
    img_input_normal = imread(input_path + input_img_num + "_roi_a.jpg");
    img_input_curve = imread(input_path + input_img_num + "_roi_b.jpg");
    img_output_contact = img_input_normal.clone();
    imshow("img_input_normal", img_input_normal);
    imshow("img_input_curve", img_input_curve);

    // ------------------------- < SWITCH CASE > ----------------------------
    // setup
    handExtraction();
    calibResult();

    // for curve estimation
    point_standard = point_hand_bottom;

    // get info about point_contact_1
    length_contact_1 = sqrt(pow(point_contact_1.x - point_hand_bottom.x, 2) + pow(point_contact_1.y - point_hand_bottom.y, 2));
    theta_contact_1 = atan2((point_contact_1.x - point_hand_bottom.x),(point_contact_1.y - point_hand_bottom.y));
    segment_contact_1 = round(length_contact_1 / length_setup * segment_N );   // where to contact
    theta_segment_contact_1 = theta_contact_1 / segment_contact_1;
    cout << "length_contact_1: " << length_contact_1 << endl;
    cout << "theta_contact_1: " << theta_contact_1 << endl;
    cout << "segment_contact_1: " << segment_contact_1 << endl;
    cout << "theta_segment_contact_1: " << theta_segment_contact_1 << endl;

    point_result_estimation.push_back(point_standard);
    curveEstimation(0, segment_N, 0, theta_segment_setup);  


    // ------------------------- < RESULT > ----------------------------
    img_output_est = img_input_normal.clone();
    img_input_curve = imread(input_path + input_img_num + "_roi_b.jpg");
    img_output_curve = img_input_curve.clone();
    
    for (int i = 0; i < point_result_estimation.size(); i++)
    {
        circle(img_output_curve, point_result_estimation[i],  8, Scalar(255,   255,   255), -1);
        circle(img_output_curve, point_result_estimation[i],  6, Scalar(255,   0,   0), -1);
        circle(img_output_est, point_result_estimation[i],  8, Scalar(255,   255,   255), -1);
        circle(img_output_est, point_result_estimation[i],  6, Scalar(255,   0,   0), -1);
    }
    imshow("img_output_curve", img_output_curve);
    imshow("img_output_est", img_output_est);
    
    // ------------------------- < VALIDITATION > ----------------------------
    Mat img_mask_valid = createMaskImg(img_input_curve, 0, 179, 0, 255, 0, 85);
    imshow("img_mask_valid", img_mask_valid);

    // findContours from img_mask
    vector<Point> contours_valid;    
    contours_valid = getContours(img_mask_valid);
    
    // find bottom and left endpoint
    int n_left = 0;
    int n_bottom = 0;

    int n_left_prev = 1000;
    int n_bottom_prev = 1000;
    for (int i = 0; i < contours_valid.size(); i++)
    {
        if(contours_valid[i].y < n_bottom_prev)
        {
            n_bottom_prev = contours_valid[i].x;
            n_bottom = i;
        }
        if(contours_valid[i].x < n_left_prev || contours_valid[i].y > 5000)
        {
            n_left_prev = contours_valid[i].x;
            n_left = i;
        }
    }

    // get points between n_left and n_bottom
    Mat img_input_valid = img_input_curve.clone();
    Mat img_overlap_valid = img_output_curve.clone();
    cout << "n_bottom = " << n_bottom << ", n_left: " << n_left << endl;
    
    // draw debug
    Point valid_start = point_hand_bottom;
    vector<Point> point_valification;

    point_valification.push_back(valid_start);
    circle(img_input_valid, valid_start,  8, Scalar(255,  255,   255), -1);
    circle(img_input_valid, valid_start,  6, Scalar(0,  255,   0), -1);
    circle(img_overlap_valid, valid_start,  8, Scalar(255,  255,   255), -1);
    circle(img_overlap_valid, valid_start,  6, Scalar(0,  255,   0), -1);
    circle(img_input_valid, contours_valid[n_left],  6, Scalar(255,  255,   0), -1);

    for(int i = 0; i <= n_left; i++)
    {
        Point valid_now = contours_valid[i];
        
        double valid_length = sqrt(pow(valid_start.x - valid_now.x ,2) + pow(valid_start.y - valid_now.y, 2));
        if(valid_length > length_segment)
        {
            circle(img_input_valid, contours_valid[i],  8, Scalar(255,  255,   255), -1);
            circle(img_input_valid, contours_valid[i],  6, Scalar(0,  255,   0), -1);

            circle(img_overlap_valid, contours_valid[i],  8, Scalar(255,  255,   255), -1);
            circle(img_overlap_valid, contours_valid[i],  6, Scalar(0,  255,   0), -1);
            valid_start = contours_valid[i];
            point_valification.push_back(contours_valid[i]);
        }
    }

    if(point_valification.size() < (segment_N + 1))
    {
        point_valification.push_back(contours_valid[n_left]);
        circle(img_overlap_valid, contours_valid[n_left],  8, Scalar(255,  255,   255), -1);
        circle(img_overlap_valid, contours_valid[n_left],  6, Scalar(0,  255,   0), -1);
    }

    imshow("valid", img_input_valid);
    imshow("valid overlap", img_overlap_valid);

    // valification output
    std::ofstream csv_writing_file;
    std::string filename = output_path + "result_" + to_string(segment_N) + ".csv";
    csv_writing_file.open(filename, std::ios::app);

    csv_writing_file << "estimation.x" << "," << "estimation.y" << "," << "valification.x" << "," << "valification.y" << "," << "diff" << endl;
    for(int i = 0; i < point_valification.size(); i++)
    {
        double diff = sqrt(pow(point_result_estimation[i].x - point_valification[i].x ,2) + pow(point_result_estimation[i].y - point_valification[i].y ,2));

        csv_writing_file << point_result_estimation[i].x 
                            << "," 
                            << point_result_estimation[i].y 
                            << "," 
                            << point_valification[i].x 
                            << "," 
                            << point_valification[i].y
                            << ","
                            << diff        
                            << endl; 
    }
    csv_writing_file.close();

        // ------------------------- < SAVE > ----------------------------
    imwrite(output_path + input_img_num + "_" + to_string(segment_N) + "_input_normal_" + ".jpg", img_input_normal);
    imwrite(output_path + input_img_num + "_" + to_string(segment_N) + "_input_curve_" + ".jpg", img_input_curve);

    imwrite(output_path + input_img_num + "_" + to_string(segment_N) + "_mask_hand_" + ".jpg", img_mask_hand);

    imwrite(output_path + input_img_num + "_" + to_string(segment_N) + "_output_contact" + ".jpg", img_output_contact);
    imwrite(output_path + input_img_num + "_" + to_string(segment_N) + "_output_est" + ".jpg", img_output_est);
    imwrite(output_path + input_img_num + "_" + to_string(segment_N) + "_output_curve" + ".jpg", img_output_curve);

    imwrite(output_path + input_img_num + "_" + to_string(segment_N) + "_output_valid" + ".jpg", img_input_valid);
    imwrite(output_path + input_img_num + "_" + to_string(segment_N) + "_output_valid_overlap" + ".jpg", img_overlap_valid);


    // ------------------------- < END > ----------------------------
    int key = waitKey(0);
    if (key == 'q')
        return 0;


    return 0;
}
