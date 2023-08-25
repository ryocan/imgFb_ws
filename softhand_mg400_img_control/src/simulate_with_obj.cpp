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

vector<Point> getContours(Mat img_mask)
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
    // ------------------------- < setup > ----------------------------
    // ros setup
    ros::init(argc, argv, "simulate_with_obj");

    // input img
    Mat img_input_normal = imread("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/input_images/with obj/input_1a.jpg");
    imshow("img_input_normal", img_input_normal);
    Mat img_output_contact = img_input_normal.clone();

    // ------------------------- < obj > ----------------------------
    // convert to mask img
    Mat img_mask_obj = createMaskImg(img_input_normal, 0, 15, 0, 255, 80, 255);
    imshow("img_mask_obj", img_mask_obj);

    // get contour
    vector<Point> contours_obj;    
    contours_obj = getContours(img_mask_obj);

    // find endpoint
    int Po_N = 0;  // endpoint of the object in the right side
    int Po_x_prev = 0;

    for (int i = 0; i < contours_obj.size(); i++)
    {
        if(contours_obj[i].x > Po_x_prev )
        {
            Po_x_prev = contours_obj[i].x;
            Po_N = i;
        }
    }
    circle(img_output_contact, contours_obj[Po_N],  8, Scalar(255,   255,   255), -1);
    circle(img_output_contact, contours_obj[Po_N],  6, Scalar(  0,   255,     0), -1);
    // imshow("img_output_contact", img_output_contact);

    // ------------------------- < hand > ----------------------------
    Mat img_mask_hand = createMaskImg(img_input_normal, 0, 179, 0, 255, 0, 65);
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

    // find endpoint
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
        if(contours_hand[i].y < contours_hand[Pht_N].y - 2)
        {
            if(contours_hand[i].x < Phtl_x_prev)
            {
                Phtl_x_prev = contours_hand[i].x;
                Phtl_N = i;
            }
        }
    }

    // pointout
    circle(img_output_contact, contours_hand[Phbl_N],  8, Scalar(255,   255,   255), -1);
    circle(img_output_contact, contours_hand[Phbl_N],  6, Scalar(255,   0,   0), -1);
    circle(img_output_contact, contours_hand[Phtl_N],  8, Scalar(255,   255,   255), -1);
    circle(img_output_contact, contours_hand[Phtl_N],  6, Scalar(255,   0,   0), -1);
    imshow("img_output_contact", img_output_contact);

    // ------------------------- < divide hand into segment > ----------------------------
    int segment_N = 5;
    double setup_angle = -0.471537;
    double setup_length = contours_hand[Phtl_N].y - contours_hand[Phbl_N].y;
    cout << "setup_length: " << setup_length << endl;

    double segment_length_init = setup_length / segment_N;
    cout << "setup_length_init: " << segment_length_init << endl;

    // ------------------------- < where to contact > ----------------------------
    double length_contact = contours_obj[Po_N].y - contours_hand[Phbl_N].y;
    cout << "length_contact: " << length_contact << endl;

    double length_contact_acc = 0;  // acc = accumulate
    int contact_N = 0;
    for (int i = 0; i < segment_N; i++)
    {
        length_contact_acc += segment_length_init;
        if ( length_contact < length_contact_acc )
        {
            contact_N = i;
            cout << "contact segment: " << i << endl;
            break;
        } 
    }

    // ------------------------- < curve estimation > ----------------------------
    Mat img_output_est = img_input_normal.clone();
    
    vector<Point> estimated_points; // results

    Point point_standard;
    point_standard = contours_hand[Phbl_N];
    estimated_points.push_back(point_standard);

    // straightforward
    for (int i = 0; i <= contact_N; i++)
    {
        double y = point_standard.y + segment_length_init;
        point_standard.y = y;
        estimated_points.push_back(point_standard);
    }
    
    // remain segment and bending angle
    int segment_remain_N = segment_N - (contact_N + 1);
    double segment_angle_init = setup_angle / segment_remain_N;
    double segment_angle_part = 0;
    
    // curve estimation
    for (int i = 0; i < segment_remain_N; i++)  
    {
        // bending angle of each segment
        segment_angle_part = segment_angle_init * (2 * i + 1);
        cout << "-----" << endl;
        cout << "angle_part: " << segment_angle_part << endl;
        
        // calculated slope of the line from bending angle
        double slope = tan(3.14/2 - segment_angle_part);
        cout << "slope: " << slope << endl; 
        cout << "point_standard: " << point_standard << endl; 

        // estimate curve end point
        int j = 0;
        while(true)
        {
            int x = point_standard.x - j;
            double y = slope * (x - point_standard.x) + point_standard.y;
            if( sqrt(pow(x - point_standard.x, 2) + pow(y - point_standard.y, 2)) > segment_length_init)
            {
                estimated_points.push_back(Point(x, (int)y));
                point_standard = Point(x, (int)y);
                cout << "-----" << endl;
                break;
            }

            j++;
        }

    }

    // show result
    for (int i = 0; i < estimated_points.size(); i++)
    {
        circle(img_output_est, estimated_points[i],  8, Scalar(255,   255,   255), -1);
        circle(img_output_est, estimated_points[i],  6, Scalar(255,   0,   0), -1);
    }
    imshow("img_output_est", img_output_est);

    // ------------------------- < check result > ----------------------------
    Mat img_input_curve = imread("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/input_images/with obj/input_1b.jpg");
    Mat img_output_curve = img_input_curve.clone();
    for (int i = 0; i < estimated_points.size(); i++)
    {
        circle(img_output_curve, estimated_points[i],  8, Scalar(255,   255,   255), -1);
        circle(img_output_curve, estimated_points[i],  6, Scalar(255,   0,   0), -1);
    }
    imshow("img_output_curve", img_output_curve);

    // ------------------------- < output > ----------------------------
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simulate_with_obj/img_input_normal_" + to_string(segment_N) + ".jpg", img_input_normal);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simulate_with_obj/img_mask_obj_" + to_string(segment_N) + ".jpg", img_mask_obj);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simulate_with_obj/img_mask_hand_" + to_string(segment_N) + ".jpg", img_mask_hand);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simulate_with_obj/img_output_contact" + to_string(segment_N) + ".jpg", img_output_contact);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simulate_with_obj/img_output_est" + to_string(segment_N) + ".jpg", img_output_est);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/simulate_with_obj/img_output_curve" + to_string(segment_N) + ".jpg", img_output_curve);

    // -----------------------------------------------------
    int key = waitKey(0);
    if (key == 'q')
        return 0;


    return 0;
}