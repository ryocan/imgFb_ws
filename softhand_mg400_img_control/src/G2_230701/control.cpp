// for image processing
#include "softhand_mg400_img_control/control.h"

/*************************************************************
 * @Function
 *    get current state
**************************************************************/
void ControlCommand::mgCb(const mg400_bringup::ToolVectorActualConstPtr& msg)
{
    mg_x = msg->x;
    mg_y = msg->y;
    mg_z = msg->z;
    mg_r = msg->r;

    // ROS_INFO("mg_x = %f, mg_y = %f, mg_z = %f, mg_r = %f", mg_x, mg_y, mg_z, mg_r);

}

/*************************************************************
 * @Function
 *    Dobot MG400: enable
 * @Details
 *    
**************************************************************/
bool ControlCommand::mg_enable()
{
  mg400_bringup::EnableRobot set_enable;
  int response;
  
  
  response = mg_enable_client.call(set_enable);
  ROS_INFO("mg_enable:response -> %d", response);

  return true;
}

/*************************************************************
 * @Function
 *    Dobot MG400: disable
 * @Details
 *    
**************************************************************/
bool ControlCommand::mg_disable()
{
  mg400_bringup::DisableRobot set_disable;
  
  mg_disable_client.call(set_disable);
  return true;
}

/*************************************************************
 * @Function
 *    Dobot MG400: MovL
 * @Details
 *    
**************************************************************/
bool ControlCommand::mg_movl(float x, float y, float z, float r)
{
  mg400_bringup::MovL set_mg_command;
  
  set_mg_command.request.x = x;
  set_mg_command.request.y = y;
  set_mg_command.request.z = z;
  set_mg_command.request.r = r;

  mg_movl_client.call(set_mg_command);

  return true;
}

/*************************************************************
 * @Function
 *    get param in imgProc.h from param.yaml
**************************************************************/
bool ControlCommand::getParam()
{
    // hsv param: hand
    cnh_.getParam("soft_img_control/H_MIN_HAND", H_MIN_HAND);
    cnh_.getParam("soft_img_control/H_MAX_HAND", H_MAX_HAND);
    cnh_.getParam("soft_img_control/S_MIN_HAND", S_MIN_HAND);
    cnh_.getParam("soft_img_control/S_MAX_HAND", S_MAX_HAND);
    cnh_.getParam("soft_img_control/V_MIN_HAND", V_MIN_HAND);
    cnh_.getParam("soft_img_control/V_MAX_HAND", V_MAX_HAND);

    // hsv param: obj
    cnh_.getParam("soft_img_control/H_MIN_OBJ", H_MIN_OBJ);
    cnh_.getParam("soft_img_control/H_MAX_OBJ", H_MAX_OBJ);
    cnh_.getParam("soft_img_control/S_MIN_OBJ", S_MIN_OBJ);
    cnh_.getParam("soft_img_control/S_MAX_OBJ", S_MAX_OBJ);
    cnh_.getParam("soft_img_control/V_MIN_OBJ", V_MIN_OBJ);
    cnh_.getParam("soft_img_control/V_MAX_OBJ", V_MAX_OBJ);

    // display info
    ROS_INFO("------ H_MIN_HAND = %d, H_MAX_HAND = %d, S_MIN_HAND = %d, S_MAX_HAND = %d, V_MIN_HAND = %d, V_MAX_HAND = %d,",
        H_MIN_HAND, H_MAX_HAND, S_MIN_HAND, S_MAX_HAND, V_MIN_HAND, V_MAX_HAND);
    ROS_INFO("------ H_MIN_OBJ = %d, H_MAX_OBJ = %d, S_MIN_OBJ = %d, S_MAX_OBJ = %d, V_MIN_OBJ = %d, V_MAX_OBJ = %d,",
        H_MIN_OBJ, H_MAX_OBJ, S_MIN_OBJ, S_MAX_OBJ, V_MIN_OBJ, V_MAX_OBJ);


    return true;
}
/*************************************************************
 * @Function
 *    Dobot MG400: MovL
 * @Details
 *    
**************************************************************/
bool ControlCommand::imgexec()
{
    // debug
    if (!img_src.empty())
    {
        imshow("img_src", img_src);
        img_output = img_src.clone();
        img_black = Mat::zeros(img_src.size(), img_src.type());

        // create mask img
        img_mask_obj = createMaskImg(img_src, H_MIN_OBJ, H_MAX_OBJ, S_MIN_OBJ, S_MAX_OBJ, V_MIN_OBJ, V_MAX_OBJ);
        img_mask_hand = createMaskImg(img_src, H_MIN_HAND, H_MAX_HAND, S_MIN_HAND, S_MAX_HAND, V_MIN_HAND, V_MAX_HAND);

        // get contours
        contours_obj = getContours(img_mask_obj, mode_obj);
        contours_hand = getContours(img_mask_hand, mode_hand);

        // calc centroid
        centroid_obj = calcCentroid(contours_obj, mode_obj);
        centroid_hand = calcCentroid(contours_hand, mode_hand);

        // extract Hand side
        contours_obj_endpoint = findObjEndPoint(contours_obj , centroid_obj );
        contours_hand_endpoint = findHandEndPoint(contours_hand , centroid_hand );

        // contours deform
        contours_hand_deform = calcDeformation(contours_hand_endpoint);

        // draw contact position
        img_black2 = Mat::zeros(img_src.size(), img_src.type());
        Point start(contours_hand_deform[(contours_hand_deform.size()-1)].x, 1);
        Point end(contours_hand_deform[(contours_hand_deform.size()-1)].x, 1023);
        line(img_black2, start, end, Scalar(0, 255, 0), 1);
        imshow("img_black2", img_black2);
        Mat img_mask_hand2 = createMaskImg(img_black2, 10, 179, 0, 255, 0, 255);
        vector<Vec4i> hierarchy2;
        cv::findContours(img_mask_hand2, contours_hand2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // draw contours and detect contact point
        drawContours(contours_obj, img_mask_obj, mode_obj);
        drawContours(contours_hand2, img_mask_hand2, mode_hand);
        for(int i = 0; i < contours_hand_contact.size(); i++)
        {
            circle(img_output, contours_hand_contact[i],  10, Scalar(255, 255, 255), 8, 8);
            circle(img_output, contours_hand_contact[i],   8, Scalar(  0, 255,   0), 8, 8);
            circle(img_output, contours_hand_contact[i],   2, Scalar(255, 255, 255), 8, 8);
        }
        imshow("img_black", img_black);

        // 
        target_point = determineTargetPoint(contours_hand_endpoint, contours_hand_deform, contours_obj_endpoint);
        cout << "target_point :" << target_point << endl;

        // display
        imshow("img_output", img_output);

        flag = 2;
        imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/img_output.jpg",img_output);
    }
    
    return true;
}

/*************************************************************
 * @Function
 *    Dobot MG400: MovL
 * @Details
 *    
**************************************************************/
bool ControlCommand::armexec()
{
    if( mg_x != 0 )
    {
        ROS_INFO("MG400 will move in 5 seconds...");
        sleep(5);

        // calc movement of mg400
        double command_pixel = contours_hand_endpoint[0].y - target_point.y;
        double command_mg400 = command_pixel / 4.;

        cout << "command :" << mg_z  << endl;
        double mg_z_command = command_mg400 + mg_z;
        cout << "command :" << command_mg400  << endl;
        cout << "command :" << mg_z_command  << endl;

        mg_movl(mg_x, mg_y, mg_z_command, mg_r);

        ROS_INFO("Image processing will restart in 3 seconds...");
        sleep(3);        
        flag = 3;
    }

    return true;
}

/*************************************************************
 * @Function
 *    Dobot MG400: MovL
 * @Details
 *    
**************************************************************/
bool ControlCommand::showexec()
{
    imshow("img_src", img_src);
    img_output = img_src.clone();
    contours_hand_deform = calcDeformation(contours_hand_endpoint);
    imshow("img_output", img_output);

    return true;
}

/*************************************************************
 * @Function
 *    Dobot MG400: MovL
 * @Details
 *    
**************************************************************/
bool ControlCommand::exec()
{


    int key = waitKey(1);
    if (key == 'q')
    {
        return false;
    }
    

    switch (flag)
    {
        case 1:
            imgexec();
            break;
        
        case 2:
            armexec();
            break;

        case 3:
            showexec();
            break;
        
        default:
            break;
    }


    
    
    return true;
}