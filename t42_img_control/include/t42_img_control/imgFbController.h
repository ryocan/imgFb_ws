/*******************************************************
@Comment
    imgFbController.h
*******************************************************/
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// for image processing
#include "t42_img_control/imgProc.h"

// about dynamixel
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

// standard libraries
#include <math.h>
#include <sys/stat.h>   //mkdir
#include <std_msgs/String.h>


//-----------------------------------------------------
// CLASS
//-----------------------------------------------------
class imgFbController : public imgProc
{
private:
    /*--- ros setup ---*/
    ros::NodeHandle nh;
    ros::ServiceClient dynamixel_client;

    /*--- parameter from param.yaml ---*/
    int GOAL_VELOCITY;
    int DEFORMATION_TH;
    int OCCLUSION_TH;

    /*------------- for graspDetection -------------*/
    // for IfG1
    vector<Point> centroid_hand_prev;
    double hand_l_shift = 0.;
    double hand_r_shift = 0.;

    // for IfG2
    vector<Point> centroid_obj_prev;
    double centroid_obj_v_shift = 0.;

    // for IfG3
    Mat img_gray;
    Mat img_gray_prev;
    vector<Point2f> opticalflow_obj;
    vector<Point2f> opticalflow_obj_prev;
    vector<Point2f> opticalflow_obj_extract;
    double deform_total = 0.;
    double deform_ratio = 0.;

    // for IfG4
    double area_hand_init = 0.;

    // whole
    int frame_count_gd = 0;
    
    /*------------- for drop and rotation detection -------------*/
    int frame_count_dard = 0;
    double drop_dist_init = 0.;
    double drop_dist = 0.;
    double dropped_time = 0.;

    double rotate_height = 0.;
    double rotate_height_prev = 0.;
    double rotate_diff = 0.;
    double rotate_total = 0.;


    /*--- for conditional branch using in exec ---*/
    int flag = 0;

    /*--- functions ---*/   
    // for contact detection
    bool contactDetection();

    // for grasp detection
    bool graspDetection();

    // for drop and rotation detection
    bool dropAndRotateDetection();

    // initialize
    void initialize();

    
public:
    imgFbController()
    {
        // service client for dynamixel
        dynamixel_client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
        // execute whole detection program
        // exec();
    }

    ~imgFbController()
    {
    }

    /*--- functions ---*/
    // get parameter in imgProc.h from param.yaml
    bool getParam();

    // dynamixel control using service communication 
    bool dynamixelCommandMsg(string command, uint8_t id, string addr_name, int32_t value);

    // execute whole detection program
    bool exec();
};