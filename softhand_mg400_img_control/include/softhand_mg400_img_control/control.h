/*******************************************************
@Comment
    cv_controlCommand.h
*******************************************************/
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// for image processing
#include "softhand_mg400_img_control/imgProc.h"

// about mg400
#include "mg400_bringup/EnableRobot.h"
#include "mg400_bringup/DisableRobot.h"
#include "mg400_bringup/MovL.h"
#include <mg400_bringup/ToolVectorActual.h>

#include <math.h>
#include <sys/stat.h>   //mkdir
#include <std_msgs/String.h>


//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
#define FPS 25
//-----------------------------------------------------
// CLASS
//-----------------------------------------------------
class ControlCommand : public imgProc
{
private:
    /*************************
     * for ROS
    *************************/
    ros::NodeHandle cnh_;
    ros::NodeHandle pnh_;

    // client
    ros::ServiceClient dyClient;
    ros::ServiceClient mg_enable_client; 
    ros::ServiceClient mg_disable_client; 
    ros::ServiceClient mg_movl_client;

    int flag = 1;
    Point target_point;
    Mat img_black2;
    /*************************
     * for Dobot MG400
    *************************/
    float mg_x;
    float mg_y;
    float mg_z;
    float mg_r;
    /*************************
     * function
    *************************/

    // callback
    void mgCb(const  mg400_bringup::ToolVectorActualConstPtr& msg);

    // exec
    bool imgexec();
    bool armexec();
    bool showexec();

    // mg400
    ros::Subscriber tool_vector_actual_sub_;
    bool mg_enable();
    bool mg_movl(float x, float y, float z, float r);


public:
    bool mg_disable();
    ControlCommand()
    {   
        // dobot topic
        tool_vector_actual_sub_ = cnh_.subscribe("/mg400_bringup/msg/ToolVectorActual", 1, &ControlCommand::mgCb, this);

        // dobot service
        mg_enable_client = cnh_.serviceClient<mg400_bringup::EnableRobot>("/mg400_bringup/srv/EnableRobot");
        mg_disable_client = cnh_.serviceClient<mg400_bringup::DisableRobot>("/mg400_bringup/srv/DisableRobot");
        mg_movl_client = cnh_.serviceClient<mg400_bringup::MovL>("/mg400_bringup/srv/MovL");
        mg_enable();

        exec(); // if using camera
    }

    ~ControlCommand()
    {
        mg_disable();
    }

    // get param from yaml
    bool getParam();

    //  exec
    bool exec();
};