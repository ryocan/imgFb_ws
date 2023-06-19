//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "softhand_mg400_img_control/imgProc.h"


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

    // roi
    int roi_x = img_src.cols / 2;
    int roi_y = 0;
    int roi_width = img_src.cols / 2;
    int roi_height = img_src.rows;

    cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
    img_src = img_src(roi);
}

/*************************************************************

 * @Function
 *    extract region based on its color
**************************************************************/
Mat imgProc::createMaskImg(Mat img, int H_MIN, int H_MAX, int S_MIN, int S_MAX, int V_MIN, int V_MAX)
{
    Mat img_mask = Mat::zeros(img_src.size(), img_src.type());

    // convert to HSV image
    Mat img_hsv;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    // extract region based on HSV parameter
    Scalar Lower(H_MIN, S_MIN, V_MIN);
    Scalar Upper(H_MAX, S_MAX, V_MAX);
    inRange(img_hsv, Lower, Upper, img_mask);

    // erode and dilate
    // Mat element4 = (Mat_<uchar>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
    // erode(img_mask, img_mask, element4, Point(-1, -1), 1);
    // dilate(img_mask, img_mask, element4, Point(-1, -1), 1);

    return img_mask;
}

/*************************************************************
 * @Function
 *    calculation contour from mask image
**************************************************************/
vector<vector<Point>> imgProc::getContours(Mat img_mask, string mode)
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
        double area_th = 12000.;
        for (int i = 0; i < contours.size(); i++)
        {
            area = contourArea(contours[i]);
            if (area > area_th)
                contours_output.push_back(contours[i]);
        }
    }

    // for debug and display
    // for(int i = 0; i < contours_output.size(); i++)
    //     cv::drawContours(img_output, contours_output, i, Scalar(0, 255, 0), 1, 8);

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
 *    calculate side contours
**************************************************************/
vector<Point> imgProc::findObjEndPoint(vector<vector<Point>> contours, vector<Point> centroid)
{
    // output
    vector<Point> contours_endpoint;

    // extract the contour as points within a threshold range from the endpoint
    double y_objbase = centroid[0].y;
    double y_objtop = centroid[0].y;
    int target_objbase = 0;
    int target_objtop = 0;

    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours[i].size(); j++)
        {
            // find fingertip and fingerbase
            if( contours[i][j].y >= y_objbase )
            {
                y_objbase = contours[i][j].y;
                target_objbase = j;
            }
            if( contours[i][j].y <= y_objtop )
            {
                y_objtop = contours[i][j].y;
                target_objtop = j;
            }
        }
    }

    // pushback to output contour
    contours_endpoint.push_back(contours[0][target_objbase]);
    contours_endpoint.push_back(contours[0][target_objtop]);

    // debug draw
    circle(img_output, contours_endpoint[0], 1, Scalar(255,   0,   0), 8, 8);
    circle(img_output, contours_endpoint[1], 1, Scalar(  0,   0, 255), 8, 8);


    return contours_endpoint;
}

/*************************************************************
 * @Function
 *    calculate side contours
**************************************************************/
vector<Point> imgProc::findHandEndPoint(vector<vector<Point>> contours, vector<Point> centroid)
{
    // output
    vector<Point> contours_endpoint;
    int target_fingertip = 0;
    int target_fingerbase = 0;

    // determine endpoint
    int target_endpoint = 0;
    double x_prev = centroid[0].x;

    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours[i].size(); j++)
        {
            if (contours[i][j].x < x_prev)
            {
                target_endpoint = j; // contours[i][target] is encpoint
                x_prev = contours[i][j].x;
            }
        }
    }

    // extract the contour as points within a threshold range from the endpoint
    double x_th = 5.; 
    double y_fingertip = centroid[0].y;
    double y_fingerbase = centroid[0].y;

    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours[i].size(); j++)
        {
            if (contours[i][j].x <= (contours[i][target_endpoint].x + x_th))  // if it is within the threshold range
            {
                // find fingertip and fingerbase
                if( contours[i][j].y >= y_fingertip )
                {
                    y_fingertip = contours[i][j].y;
                    target_fingertip = j;
                }
                if( contours[i][j].y <= y_fingerbase )
                {
                    y_fingerbase = contours[i][j].y;
                    target_fingerbase = j;
                }
            }
        }
    }

    // pushback to output contour
    contours_endpoint.push_back(contours[0][target_fingertip]);
    contours_endpoint.push_back(contours[0][target_fingerbase]);

    // debug draw
    circle(img_output, contours_endpoint[0], 1, Scalar(255,   0,   0), 8, 8);
    circle(img_output, contours_endpoint[1], 1, Scalar(  0,   0, 255), 8, 8);


    return contours_endpoint;
}

/*************************************************************
 * @Function
 *    calculate deformation
**************************************************************/
vector<Point> imgProc::calcDeformation(vector<Point> contours_endpoint)
{
    // output
    vector<Point> contours_deform;

    // calc finger lemgth in initial state
    double finger_length = contours_endpoint[0].y - contours_endpoint[1].y;
    
    // draw contours
    int x = 0;
    int y = 0;
    int x_prev = 0;
    int y_prev = 0;

    int x_output = 0;
    int y_output = 0;
    double curve_length = 0.;
    double curve_length_total = 0.;
    for (int i = 0; i < 100; i++)
    {
        // Approximating the deformation with a quadratic function
        x = -i;
        y = -0.0171 * pow(x, 2) - (3.4666 * x) + 5.823;

        // calculation of curve length
        if ( i != 0)
            curve_length = sqrt( pow(abs(x - x_prev), 2) + pow(abs(y - y_prev), 2) );
        
        // compare length
        curve_length_total += curve_length;
        if ( curve_length_total > finger_length)
            break;
        
        // correcting fingerbase point to be origin point
        x_output = contours_endpoint[1].x + x;
        y_output = contours_endpoint[1].y + y;
        circle(img_output, Point(x_output, int(y_output)), 1, Scalar(0, 0, 255), 2, 1);
        contours_deform.push_back(Point(x_output, int(y_output)));

        // update
        x_prev = x;
        y_prev = y;
    }

    return contours_deform;
}

/*************************************************************
 * @Function
 *    drawContours
**************************************************************/
void imgProc::drawContours(vector<vector<Point>> contours, Mat img_mask, string mode)
{
    // declare for LineIterator
    Point li_start;
    Point li_goal;
    
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
                    contours_hand_contact.push_back(Point(li_point[k].x, li_point[k].y));
                }
            }
        }
    }
}

/*************************************************************
 * @Function
 *    exec
**************************************************************/
Point imgProc::determineTargetPoint(vector<Point> contours_hand_end, vector<Point> contours_deform, vector<Point> contours_obj_end)
{
    Point target_point(0, 0);
    double area = 0.;
    double prev_area = 0.;

    // set origin
    for (int k = 0; k < contours_hand_contact.size(); k++)
    {
        vector<Point> contours_output;
        int y_move = contours_hand_contact[k].y - contours_deform[contours_deform.size() - 1].y;

        // pushback arm contours 
        int x = 0;
        int y = 0;
        for (int i = 1; i < contours_deform.size(); i++)
        {
            x = contours_deform[contours_deform.size() - i].x;
            y = contours_deform[contours_deform.size() - i].y + y_move;
            contours_output.push_back(Point(x, y));
        }

        // 
        int j_up = 0;
        int j_down = 0;
        if ( contours_output[contours_output.size() - 1].y <  contours_obj_end[1].y)
        {   
            contours_output.push_back(Point(contours_hand_contact[0].x, contours_output[contours_output.size() - 1].y));

            for ( int i = 0; i < contours_obj[0].size(); i++)
            {
                if ( contours_obj[0][i].x - 1 == contours_output[contours_output.size() - 1].y )
                    j_up = i;
                if (contours_obj[0][i].x - 1 == contours_hand_contact[1].y)
                    j_down = i;
            }

            // pushback contours
            for ( int j = j_up; j_down < j; j--)
                contours_output.push_back(contours_obj[0][j]);


        }
        else
        {
            // find contours
            for ( int i = 0; i < contours_obj[0].size(); i++)
            {
                if ( contours_obj[0][i].y - 1 == contours_output[contours_output.size() - 1].y )
                    j_up = i;
                if (contours_obj[0][i].y - 1 == contours_hand_contact[k].y)
                    j_down = i;
            }

            // pushback contours
            for ( int j = j_up; j_down < j; j--)
                contours_output.push_back(contours_obj[0][j]);

        }
        
        vector<vector<Point>> contours_output2;
        contours_output2.push_back(contours_output);

        for (int i = 0; i < contours_output2.size(); i++)
            cv::drawContours(img_output, contours_output2, i, Scalar(255, 255, 255), -1);
        
        // find best position
        area = contourArea(contours_output);
        if ( area < prev_area)
            target_point = contours_hand_contact[k];
        prev_area = area;
    }
    
    return target_point;

}