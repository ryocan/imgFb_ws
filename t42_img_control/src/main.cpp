#include "t42_img_control/imgFbController.h"

int main(int argc, char** argv)
{
    // node setup
    ros::init(argc, argv, "img_fb_controller");
    
    // define imgFbController Class as ifc
    imgFbController ifc;
    imgProc ip;

    // get param from yaml
    if(!ifc.getParam())
    {
        ROS_WARN("Failed to get parameter from yaml file");
        return false;
    }

    // run
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        if(!ifc.exec())
        {
            ROS_WARN("Program stopped");
            break;
        }
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}