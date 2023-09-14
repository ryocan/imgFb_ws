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
    ros::init(argc, argv, "setup");

    // input img
    Mat img_input = imread("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/input_images/with obj/pear/poseB/121_roi_b.jpg");
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
    int Pl_N = 0;

    int Pt_y_prev = 0;
    int Pb_y_prev = 320;
    int Pl_x_prev = 640;
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
        if(contours[i].x < Pl_x_prev)
        {
            Pl_x_prev = contours[i].x;
            Pl_N = i;
        }
    }

    // pointout
    circle(img_output, contours[Pb_N],  8, Scalar(255,   255,   255), -1);
    circle(img_output, contours[Pb_N],  6, Scalar(255,   0,   0), -1);

    circle(img_output, contours[Pl_N],  8, Scalar(255, 255, 255), -1);
    circle(img_output, contours[Pl_N],  6, Scalar(  0,   0, 255), -1);

    cout << contours[Pb_N] << endl;
    cout << contours[Pl_N] << endl;
    imshow("img_output", img_output);

    // arctan
    double theta = atan2((contours[Pl_N].x - contours[Pb_N].x),(contours[Pl_N].y - contours[Pb_N].y));
    double angle = theta * 180 / 3.14;
    cout << "theta: " << theta << endl;
    cout << "angle: " << angle << endl;



    // ------ draw -------- //
    Mat img_output2 = img_output.clone();
    line(img_output2, contours[Pb_N], contours[Pl_N], Scalar(255, 255, 255), 3, 8);
    line(img_output2, contours[Pb_N], contours[Pl_N], Scalar(30, 255, 30), 2, 8);
    line(img_output2, contours[Pb_N], Point(contours[Pb_N].x, contours[Pl_N].y), Scalar(255, 255, 255), 3, 8);
    line(img_output2, contours[Pb_N], Point(contours[Pb_N].x, contours[Pl_N].y), Scalar(30, 255, 30), 2, 8);
    putText(img_output2, "rad: "+to_string(theta), Point(img_output2.cols - 150, img_output2.rows - 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    putText(img_output2, "deg: "+to_string(angle), Point(img_output2.cols - 150, img_output2.rows - 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

    imshow("img_output2", img_output2);
    
    // output
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/setup/img_input.jpg", img_input);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/setup/img_mask_hand.jpg", img_mask_hand);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/setup/img_output.jpg", img_output);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/setup/img_output2.jpg", img_output2);

    int key = waitKey(0);
    if (key == 'q')
        return 0;


    return 0;
}