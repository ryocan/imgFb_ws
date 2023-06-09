//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "t42_img_control/imgProc.h"


/*************************************************************
 * @Function
 *    subscribe image from camera
**************************************************************/
void imgProc::imgCb(const sensor_msgs::ImageConstPtr& msg)
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

    // imshow("img_Src", img_src);
    // waitKey(1);
}


/*************************************************************
 * @Function
 *    extract region based on its color
**************************************************************/
Mat imgProc::extractRegion(int H_MIN, int H_MAX, int S_MIN, int S_MAX, int V_MIN, int V_MAX)
{
    Mat img_mask = Mat::zeros(img_src.size(), img_src.type());

    // convert to HSV image
    Mat img_hsv;
    cvtColor(img_src, img_hsv, COLOR_BGR2HSV);

    // extract region based on HSV parameter
    Scalar Lower(H_MIN, S_MIN, V_MIN);
    Scalar Upper(H_MAX, S_MAX, V_MAX);
    inRange(img_hsv, Lower, Upper, img_mask);

    return img_mask;
}


/*************************************************************
 * @Function
 *    calculation contour from mask image
**************************************************************/
vector<vector<Point>> imgProc::calcContours(Mat img_mask, string mode)
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

    // for debug and display
    // for(int i = 0; i < contours_output.size(); i++)
    //     cv::drawContours(img_output, contours_output, i, Scalar(255,255,255), 1, 8);

    return contours_output;
}


/*************************************************************
 * @Function
 *    calculate centroid 
**************************************************************/
vector<Point> imgProc::calcCentroid(vector<vector<Point>> contours, string mode)
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


/*************************************************************
 * @Function
 *    draw contours using Line Iterator for contact detection 
**************************************************************/
void imgProc::drawContours(vector<vector<Point>> contours, Mat img_mask, string mode)
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
                
                for (int n = 0; n <= line_width; n++)
                {
                    if (mode == "obj")
                        img_black.at<Vec3b>(li_point[k].y, li_point[k].x + sign_x * n) = Vec3b(0, 0, 255);
                    else if (mode == "hand")
                        img_black.at<Vec3b>(li_point[k].y, li_point[k].x + sign_x * n)[0] += 255; //overwrite on Hand's line


                    if ((img_black.at<Vec3b>(li_point[k].y, li_point[k].x + sign_x * n)) == Vec3b(255, 0, 255))
                    {
                        if (li_point[k].x < centroid_obj[0].x) // to judge which hand is touched
                            contact_hand_L++;
                        else 
                            contact_hand_R++;
                    }
                }
            }
        }
    }
}


/*************************************************************
 * @Function
 *    extracting only the available points of optical flow
**************************************************************/
vector<Point2f> imgProc::opticalflowExtract(vector<Point2f> opticalflow_prev, vector<Point2f> opticalflow, vector<uchar> status, Mat img)
{
    vector<Point2f> opticalflow_output;
    for (uint i = 0; i < opticalflow.size(); i++)
    {
        if (status[i] == 1)
        {
            double x = abs(opticalflow_prev[i].x - opticalflow[i].x);
            double y = abs(opticalflow_prev[i].y - opticalflow[i].y);

            if (x < 50 && y < 50)
            {
                opticalflow_output.push_back(opticalflow[i]);
                cv::circle(img_output, opticalflow[i], 3, Scalar(0, 0, 255), -1);
            }
        }
    }

    return opticalflow_output;
}
