#include "softhand_mg400_img_control/control.h"

int main(int argc, char** argv)
{
    // node setup
    ros::init(argc, argv, "soft_img_control");
    ControlCommand cc;

    // get param from yaml
    if(!cc.getParam())
    {
        ROS_WARN("Failed to get parameter from yaml file");
        return false;
    }

    VideoWriter writer_src("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/src.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 100, Size(640, 1024), true);

    // run
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        if (!cc.img_src.empty())
            writer_src.write(cc.img_src);

        if(!cc.exec())
        {
            ROS_WARN("Program stopped");
            break;
        }

        ros::spinOnce();
		loop_rate.sleep();
    }

    cc.mg_disable();

    return 0;
}