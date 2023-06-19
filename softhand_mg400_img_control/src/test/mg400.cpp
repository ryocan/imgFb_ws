// about mg400
#include "mg400_bringup/EnableRobot.h"
#include "mg400_bringup/DisableRobot.h"
#include "mg400_bringup/MovL.h"
#include <mg400_bringup/ToolVectorActual.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
using namespace std;

float mg_x;
float mg_y;
float mg_z;
float mg_r;
int flag = 2;

ros::Subscriber tool_vector_actual_sub_;
ros::ServiceClient mg_enable_client; 
ros::ServiceClient mg_disable_client; 


void mgCb(const mg400_bringup::ToolVectorActualConstPtr& msg)
{
    mg_x = msg->x;
    mg_y = msg->y;
    mg_z = msg->z;
    mg_r = msg->r;

    ROS_INFO("mg_x = %f, mg_y = %f, mg_z = %f, mg_r = %f", mg_x, mg_y, mg_z, mg_r);
}

bool mg_enable()
{
  mg400_bringup::EnableRobot set_enable;
  int response;
  
  response = mg_enable_client.call(set_enable);
  ROS_INFO("mg_enable:response -> %d", response);

  return true;
}

bool mg_disable()
{
  mg400_bringup::DisableRobot set_disable;
  
  mg_disable_client.call(set_disable);
  return true;
}
//////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mg400");
    ros::NodeHandle cnh_;

    // get current tool positiion
    tool_vector_actual_sub_ = cnh_.subscribe("/mg400_bringup/msg/ToolVectorActual", 1, mgCb);

    mg_enable_client = cnh_.serviceClient<mg400_bringup::EnableRobot>("/mg400_bringup/srv/EnableRobot");
    if (flag == 1)
    {
        sleep(2);
        mg_enable();
        flag = 2;
    }
        
    mg_disable_client = cnh_.serviceClient<mg400_bringup::DisableRobot>("/mg400_bringup/srv/DisableRobot");
    if (flag == 2)
    {
        sleep(5);
        mg_disable();
        flag = 3;
    }

    cout << "flag = " << flag << endl;
    // ros::spin();
}