//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "t42_img_control/imgFbController.h"


/*************************************************************
 * @Function
 *    initialize
**************************************************************/
void imgFbController::initialize()
{
    // to avoid running out of memory due to push_back, it is necessary to initialize.
    contours_obj.clear();
    contours_obj.shrink_to_fit();

    contours_hand.clear();
    contours_hand.shrink_to_fit();

    centroid_obj.clear();
    centroid_obj.shrink_to_fit();

    centroid_hand.clear();
    centroid_hand.shrink_to_fit();
}

/*************************************************************
 * @Function
 *    get param in imgProc.h from param.yaml
**************************************************************/
bool imgFbController::getParam()
{
    // goal velocity
    nh.getParam("img_fb_controller/GOAL_VELOCITY", GOAL_VELOCITY);

    // deformation
    nh.getParam("img_fb_controller/DEFORMATION_TH", DEFORMATION_TH);

    // occlusion
    nh.getParam("img_fb_controller/OCCLUSION_TH", OCCLUSION_TH);

    // hsv param: hand
    nh.getParam("img_fb_controller/H_MIN_HAND", H_MIN_HAND);
    nh.getParam("img_fb_controller/H_MAX_HAND", H_MAX_HAND);
    nh.getParam("img_fb_controller/S_MIN_HAND", S_MIN_HAND);
    nh.getParam("img_fb_controller/S_MAX_HAND", S_MAX_HAND);
    nh.getParam("img_fb_controller/V_MIN_HAND", V_MIN_HAND);
    nh.getParam("img_fb_controller/V_MAX_HAND", V_MAX_HAND);

    // hsv param: obj
    nh.getParam("img_fb_controller/H_MIN_OBJ", H_MIN_OBJ);
    nh.getParam("img_fb_controller/H_MAX_OBJ", H_MAX_OBJ);
    nh.getParam("img_fb_controller/S_MIN_OBJ", S_MIN_OBJ);
    nh.getParam("img_fb_controller/S_MAX_OBJ", S_MAX_OBJ);
    nh.getParam("img_fb_controller/V_MIN_OBJ", V_MIN_OBJ);
    nh.getParam("img_fb_controller/V_MAX_OBJ", V_MAX_OBJ);


    // display info
    ROS_INFO("------ GOAL_VELOCITY = %d", GOAL_VELOCITY);
    ROS_INFO("------ DEFORMATION_TH = %d", DEFORMATION_TH);
    ROS_INFO("------ OCCLUSION_TH = %d", OCCLUSION_TH);
    ROS_INFO("------ H_MIN_HAND = %d, H_MAX_HAND = %d, S_MIN_HAND = %d, S_MAX_HAND = %d, V_MIN_HAND = %d, V_MAX_HAND = %d,",
        H_MIN_HAND, H_MAX_HAND, S_MIN_HAND, S_MAX_HAND, V_MIN_HAND, V_MAX_HAND);
    ROS_INFO("------ H_MIN_OBJ = %d, H_MAX_OBJ = %d, S_MIN_OBJ = %d, S_MAX_OBJ = %d, V_MIN_OBJ = %d, V_MAX_OBJ = %d,",
        H_MIN_OBJ, H_MAX_OBJ, S_MIN_OBJ, S_MAX_OBJ, V_MIN_OBJ, V_MAX_OBJ);


    return true;
}


/*************************************************************
 * @Function
 *    dynamixel control using service communication 
**************************************************************/
bool imgFbController::dynamixelCommandMsg(string command, uint8_t id, string addr_name, int32_t value)
{
    dynamixel_workbench_msgs::DynamixelCommand commandMsg;

    commandMsg.request.command = command;
    commandMsg.request.id = id;
    commandMsg.request.addr_name = addr_name;
    commandMsg.request.value = value;

    dynamixel_client.call(commandMsg);
    return true;
}


/*************************************************************
 * @Function
 *    contact detection
**************************************************************/
bool imgFbController::contactDetection()
{
    // extract region
    img_mask_obj = extractRegion(H_MIN_OBJ, H_MAX_OBJ, S_MIN_OBJ, S_MAX_OBJ, V_MIN_OBJ, V_MAX_OBJ);
    img_mask_hand = extractRegion(H_MIN_HAND, H_MAX_HAND, S_MIN_HAND, S_MAX_HAND, V_MIN_HAND, V_MAX_HAND);

    // calc contours
    contours_obj = calcContours(img_mask_obj, mode_obj);
    contours_hand = calcContours(img_mask_hand, mode_hand);

    // calc centroid
    centroid_obj = calcCentroid(contours_obj, mode_obj);
    centroid_hand = calcCentroid(contours_hand, mode_hand);

    // draw contours
    drawContours(contours_obj, img_mask_obj, "obj");
    drawContours(contours_hand, img_mask_hand, "hand");

    // display image
    imshow("img_black", img_black);

    // judge
    if ((contact_hand_L > 20) && (contact_hand_R > 20))
    {
        destroyWindow("img_black");
        flag = 2;
    }

    return true;
}


/*************************************************************
 * @Function
 *    grasp detection
**************************************************************/
bool imgFbController::graspDetection()
{
    /*------------- IfG1: shift in robootic hand's centroid -------------*/
    img_mask_hand = extractRegion(H_MIN_HAND, H_MAX_HAND, S_MIN_HAND, S_MAX_HAND, V_MIN_HAND, V_MAX_HAND);
    contours_hand = calcContours(img_mask_hand, "hand");
    centroid_hand = calcCentroid(contours_hand, "hand");
    
    if (!centroid_hand_prev.empty())
    {
        // calculation
        hand_r_shift = std::pow((centroid_hand_prev[0].x - centroid_hand[0].x), 2) + pow((centroid_hand_prev[0].y - centroid_hand[0].y), 2);
        hand_l_shift = std::pow((centroid_hand_prev[1].x - centroid_hand[1].x), 2) + pow((centroid_hand_prev[1].y - centroid_hand[1].y), 2);

        // detection
        if ((hand_r_shift <= 0.) && (hand_l_shift <= 0.) && (frame_count_gd >= 30))
        {
            ROS_INFO("grasp detected(IfG1).");
            // stop dynamixel motor
            dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
            dynamixelCommandMsg("", 2, "Goal_Velocity", 0);
            flag = 3;
            return false;
        }
    }

    /*------------- IfG2: object's v-direction movement -------------*/
    img_mask_obj = extractRegion(H_MIN_OBJ, H_MAX_OBJ, S_MIN_OBJ, S_MAX_OBJ, V_MIN_OBJ, V_MAX_OBJ);
    contours_obj = calcContours(img_mask_obj, "obj");
    centroid_obj = calcCentroid(contours_obj, "obj");

    if (!centroid_obj_prev.empty())
    {
        // calc shift
        centroid_obj_v_shift = centroid_obj_prev[0].y - centroid_obj[0].y;

        if (centroid_obj_v_shift >= 5.)
        {
            ROS_INFO("grasp detected(IfG2).");
            // stop dynamixel motor
            dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
            dynamixelCommandMsg("", 2, "Goal_Velocity", 0);
            flag = 3;
            return false;
        }
    }

    // /*------------- IfG3: object deformation -------------*/
    // variable for calcOpticalFlowPyrLK
    vector<uchar> status;
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
    
    // convert image to grayscale
    cv::cvtColor(img_src, img_gray, CV_BGR2GRAY);

    if (!img_gray_prev.empty())
    {
        // calculate LK-opticalflow
        cv::calcOpticalFlowPyrLK(img_gray_prev, img_gray, opticalflow_obj_prev, opticalflow_obj, status, err, Size(20, 20), 2, criteria);
    
        // extracting only the available points of optical flow
        opticalflow_obj_extract = opticalflowExtract(opticalflow_obj_prev, opticalflow_obj, status, img_output);

        // optical flow occurs during both object motion and deformation. 
        // to detect deformation, it is necessary to determine if there is motion present.
        double deform_l = 0.;
        double deform_r = 0.;
        for (int i = 0; i < opticalflow_obj_extract.size(); i++)
        {
            if (opticalflow_obj_extract[i].x < centroid_obj[0].x)
                deform_l += opticalflow_obj_extract[i].x - opticalflow_obj_prev[i].x;
            else
                deform_r += opticalflow_obj_extract[i].x - opticalflow_obj_prev[i].x;
        }

        double centroid_obj_x_shift = abs(centroid_obj[0].x - centroid_obj_prev[0].x);
        double deform;
        if (centroid_obj_x_shift < 2.)
            deform = abs(deform_l) + abs(deform_r);
        else
            deform = 0.;    // because object moves
        
        // calc deformation
        deform_total += deform;
        deform_ratio = deform_total / opticalflow_obj_extract.size();
        if (deform_ratio > DEFORMATION_TH)
        {
            ROS_INFO("grasp detected(IfG3).");
            // stop dynamixel motor
            dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
            dynamixelCommandMsg("", 2, "Goal_Velocity", 0);
            flag = 3;
            return false;
        }
    }

    // /*------------- IfG4: occlusion rate of robotic hand -------------*/
    double area_hand = contourArea(contours_hand[0]) + contourArea(contours_hand[1]);

    // initialize area as a standard
    if (frame_count_gd == 0)
        area_hand_init = area_hand;
    
    // calc occlusion
    double occlusion_area = area_hand_init - area_hand;
    double occlusion_rate = occlusion_area / contours_obj[0].size();
    if (occlusion_rate > OCCLUSION_TH)
    {
        ROS_INFO("grasp detected(IfG4).");
        // stop dynamixel motor
        dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
        dynamixelCommandMsg("", 2, "Goal_Velocity", 0);
        flag = 3;
        return false;
    }


    /*------------- update info -------------*/
    imshow("img_output", img_output);

    // update centroid info
    centroid_hand_prev = centroid_hand;
    centroid_obj_prev = centroid_obj;

    // update grayscale image
    img_gray_prev = img_gray.clone();

    // update opticalflow_obj_prev as a real contour
    opticalflow_obj_prev.clear();
    img_mask_obj = extractRegion(H_MIN_OBJ, H_MAX_OBJ, S_MIN_OBJ, S_MAX_OBJ, V_MIN_OBJ, V_MAX_OBJ);
    contours_obj = calcContours(img_mask_obj, "obj");
    for (int i = 0; i < contours_obj.size(); i++)
        opticalflow_obj_prev.push_back(static_cast<Point2f>(contours_obj[0][i]));

    // count frame to manage judge time
    frame_count_gd++;


    return true;
}

/*************************************************************
 * @Function
 *    drop and rotation detection
**************************************************************/
bool imgFbController::dropAndRotateDetection()
{
    /*------------- drop detection -------------*/
    // in this detection, we use centroid of object and robotic hand
    img_mask_hand = extractRegion(H_MIN_HAND, H_MAX_HAND, S_MIN_HAND, S_MAX_HAND, V_MIN_HAND, V_MAX_HAND);
    contours_hand = calcContours(img_mask_hand, "hand");
    centroid_hand = calcCentroid(contours_hand, "hand");

    img_mask_obj = extractRegion(H_MIN_OBJ, H_MAX_OBJ, S_MIN_OBJ, S_MAX_OBJ, V_MIN_OBJ, V_MAX_OBJ);
    contours_obj = calcContours(img_mask_obj, "obj");
    centroid_obj = calcCentroid(contours_obj, "obj");

    // calc average point of two hand's centroid
    Point centroid_hand_average;
    centroid_hand_average.x = (centroid_hand[0].x + centroid_hand[1].x) / 2;
    centroid_hand_average.y = (centroid_hand[0].y + centroid_hand[1].y) / 2;

    // display
    cv::circle(img_output, centroid_hand_average, 10, cv::Scalar(255, 255, 255), 8, 8);
    cv::circle(img_output, centroid_hand_average,  8, cv::Scalar( 30, 255,  30), 8, 8);
    cv::circle(img_output, centroid_hand_average,  2, cv::Scalar(255, 255, 255), 8, 8);
    cv::line(img_output, centroid_hand_average, centroid_obj[0], cv::Scalar(255, 255, 255), 8, 8, 0);
    cv::line(img_output, centroid_hand_average, centroid_obj[0], cv::Scalar(  0,   0,   0), 4, 8, 0);

    // calc drop distance
    if (frame_count_dard == 0)
        drop_dist_init = centroid_obj[0].y - centroid_hand_average.y;
    else
        drop_dist = centroid_obj[0].y - centroid_hand_average.y;
    
    // grasping force weighted by the speed of descent.
    int grasp_time = 0;
    if (frame_count_dard == 0)
        grasp_time = 10;
    else
        grasp_time = 100 / (frame_count_dard - dropped_time);
    
    // detect drop
    if ((frame_count_dard > 5) && abs((drop_dist - drop_dist) >= 15))
    {
        ROS_INFO("drop distance init = %lf", drop_dist_init);
        ROS_INFO("drop distance = %lf", drop_dist);

        // regrasping by increase grasping force
        dynamixelCommandMsg("", 1, "Goal_Velocity", GOAL_VELOCITY);
        dynamixelCommandMsg("", 2, "Goal_Velocity", GOAL_VELOCITY);
        sleep(grasp_time);
        dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
        dynamixelCommandMsg("", 2, "Goal_Velocity", 0);

        // initialize
        dropped_time = frame_count_dard;
        drop_dist_init = drop_dist;
    }

    /*------------- rotation detection -------------*/
    // detect rotation by change in rectangle height
    Rect rect = boundingRect(contours_obj[0]);
    rectangle(img_output, rect, cv::Scalar(255, 255, 255), 6);
    rectangle(img_output, rect, cv::Scalar( 10, 120, 255), 3);
    rotate_height = rect.height;

    // calc roration
    if (frame_count_dard != 0)
    {
        rotate_diff = rotate_height - rotate_height_prev;
        rotate_total += rotate_diff;
    }

    // detection
    if (abs(rotate_diff) >= 15 || abs(rotate_total) > 30)
    {
        // regrasping by increase grasping force
        dynamixelCommandMsg("", 1, "Goal_Velocity", GOAL_VELOCITY);
        dynamixelCommandMsg("", 2, "Goal_Velocity", GOAL_VELOCITY);
        sleep(grasp_time);
        dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
        dynamixelCommandMsg("", 2, "Goal_Velocity", 0);
        rotate_total = 0;
    }

    /*------------- update info -------------*/
    rotate_height_prev = rotate_height;
    frame_count_dard++;

    return true;
}

/*************************************************************
 * @Function
 *    execute whole detection program
**************************************************************/
bool imgFbController::exec()
{
    // it takes time to acquire the image.
    if (!img_src.empty())
    {

        imshow("img_src", img_src); 
        
        // image setting
        img_output = img_src.clone();
        img_black = Mat::zeros(img_src.size(), img_src.type());

        // if pressed "q", the dynamixel motor will stop
        int key = waitKey(1);
        if (key == 'q')
        {
            ROS_INFO("robotic hand will stop...");
            // dynamixel torque: off
            dynamixelCommandMsg("", 1, "Torque_Enable", 0);
            dynamixelCommandMsg("", 2, "Torque_Enable", 0);
            return false;
        }

        // initialize
        initialize();

        // conditional branching using a flag
        switch (flag)
        {
            case 0:
                ROS_INFO("flag = %d: Motor will move", flag);
                // torque on
                dynamixelCommandMsg("", 1, "Torque_Enable", 1);
                dynamixelCommandMsg("", 2, "Torque_Enable", 1);

                // set goal velocity
                dynamixelCommandMsg("", 1, "Goal_Velocity", GOAL_VELOCITY);
                dynamixelCommandMsg("", 2, "Goal_Velocity", GOAL_VELOCITY);
                flag = 1;
                break;
            
            case 1:
                contactDetection();
                break;

            case 2:
                graspDetection();
                break;

            case 3:
                dropAndRotateDetection();
                break;

            default:
                ROS_INFO("robotic hand will stop...");
                // dynamixel torque: off
                dynamixelCommandMsg("", 1, "Torque_Enable", 0);
                dynamixelCommandMsg("", 2, "Torque_Enable", 0);
                return false;
                break;
        }

        //display img_output
        imshow("img_output", img_output);
    }

    return true;
}