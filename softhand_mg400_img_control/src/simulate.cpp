/* -------------------------------------------------------------------------------------
to calculate finger length and bending angle from input image
--------------------------------------------------------------------------------------- */

//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "softhand_mg400_img_control/imgProc.h"
#include <math.h>

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

vector<Point> getContours(Mat img_mask, Mat img_output)
{
    // findContours from img_mask
    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    cv::findContours(img_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // detect based on area info
    double area = 0.;
    vector<Point> contours_output;

    // change contours calculation method
    double area_prev = 0.;
    for (int i = 0; i < contours.size(); i++)
    {
        area = contourArea(contours[i]);
        if (area > area_prev)
        {
            contours_output.clear();
            contours_output = contours[i];
            area_prev = area;
        }
    }

    return contours_output;
}

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char** argv)
{
    // ros setup
    ros::init(argc, argv, "simulate");

    // input img
    Mat img_input = imread("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/roi_normal.jpg");
    imshow("img_input", img_input);
    Mat img_output = img_input.clone();

    // convert to mask img
    Mat img_mask_hand = createMaskImg(img_input, 0, 179, 0, 255, 0, 65);
    imshow("img_mask_hand", img_mask_hand);

    // findContours from img_mask
    vector<Point> contours;    
    contours = getContours(img_mask_hand, img_output);

    // sort
    int Pt_N = 0;   // num of P_t(Point top)
    int Pb_N = 0;   // num of P_b(Point bottom)

    int Ptl_N = 0;
    int Pbl_N = 0;

    int Pt_y_prev = 0;
    int Pb_y_prev = 320;
    int Ptl_x_prev = 640;
    int Pbl_x_prev = 640;
    for (int i = 0; i < contours.size(); i++)
    {
        // find Pb
        if(contours[i].y < Pb_y_prev)
        {
            Pb_y_prev = contours[i].y;
            Pb_N = i;
        }
        if(contours[i].y > Pt_y_prev)
        {
            Pt_y_prev = contours[i].y;
            Pt_N = i;
        }
    }

    for (int i = 0; i < contours.size(); i++)
    {
        if(contours[i].y < contours[Pb_N].y + 10)
        {
            if(contours[i].x < Pbl_x_prev)
            {
                Pbl_x_prev = contours[i].x;
                Pbl_N = i;
            }
        }
        if(contours[i].y < contours[Pt_N].y - 2)
        {
            if(contours[i].x < Ptl_x_prev)
            {
                Ptl_x_prev = contours[i].x;
                Ptl_N = i;
            }
        }
    }

    // pointout
    circle(img_output, contours[Pbl_N],  8, Scalar(255,   255,   255), -1);
    circle(img_output, contours[Pbl_N],  6, Scalar(255,   0,   0), -1);
    circle(img_output, contours[Ptl_N],  8, Scalar(255,   255,   255), -1);
    circle(img_output, contours[Ptl_N],  6, Scalar(255,   0,   0), -1);
    imshow("img_output", img_output);

    Mat img_output2 = img_output.clone();

    // initialize
    double length_init = contours[Ptl_N].y - contours[Pbl_N].y;
    double angle_init = -0.471537;

    // part
    int link_num = 8;
    double length_part = length_init / link_num;
    cout << "length_part: " << length_part << endl;
    double angle_part_init = angle_init / link_num;
    double angle_part = 0;

    // divide
    vector<Point> position_link;
    Point position_standard;
    position_standard = contours[Pbl_N];

    for (int i = 0; i < link_num; i++)
    {
        angle_part = angle_part_init * (2 * i+ 1);
        cout << "angle_part: " << angle_part << endl;
        double slope = tan(3.14/2 - angle_part);
        cout << "slope: " << slope << endl; 
        cout << "position_standard: " << position_standard << endl; 

        int j = 0;
        while(true)
        {
            int x = position_standard.x - j;
            double y = slope * (x - position_standard.x) + position_standard.y;
            if( sqrt(pow(x - position_standard.x, 2) + pow(y - position_standard.y, 2)) > length_part)
            {
                position_link.push_back(Point(x, (int)y));
                line(img_output2, position_standard, Point(x, (int)y), Scalar(255, 255, 255), 3, 8);
                position_standard = Point(x, (int)y);
                cout << "-----" << endl;
                break;
            }

            j++;
        }
    }

    Mat img_input3 = imread("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/roi_close.jpg");
    Mat img_output3 = img_input3.clone();

    for (int i = 0; i < position_link.size(); i++)
    {
        circle(img_output2, position_link[i],  8, Scalar(255, 255, 255), -1);
        circle(img_output2, position_link[i],  6, Scalar(  0,   0, 255), -1);
        circle(img_output3, position_link[i],  8, Scalar(255, 255, 255), -1);
        circle(img_output3, position_link[i],  6, Scalar(  0,   0, 255), -1);
    }
    circle(img_output3, contours[Pbl_N],  8, Scalar(255,   255,   255), -1);
    circle(img_output3, contours[Pbl_N],  6, Scalar(255,   0,   0), -1);
    imshow("img_output2", img_output2);
    imshow("img_output3", img_output3);

    
    // output
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simu/img_input_" + to_string(link_num) + ".jpg", img_input);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simu/img_mask_hand_" + to_string(link_num) + ".jpg", img_mask_hand);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simu/img_output_" + to_string(link_num) + ".jpg", img_output);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simu/img_output2_" + to_string(link_num) + ".jpg", img_output2);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simu/img_output3_" + to_string(link_num) + ".jpg", img_output3);

    int key = waitKey(0);
    if (key == 'q')
        return 0;


    return 0;
}