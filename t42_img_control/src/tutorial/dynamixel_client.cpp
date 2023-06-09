/*******************************************************
dynamixel client 
*******************************************************/
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// standard libraries
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

// dynamixel service client
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

// std_msgs
#include <std_msgs/String.h>

//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;

//-----------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------
ros::ServiceClient dynamixel_client;
bool dynamixelCommandMsg(string command, uint8_t id, string addr_name, int32_t value)
{
    dynamixel_workbench_msgs::DynamixelCommand commandMsg;

    commandMsg.request.command = command;
    commandMsg.request.id = id;
    commandMsg.request.addr_name = addr_name;
    commandMsg.request.value = value;

    dynamixel_client.call(commandMsg);
    return true;

    //
}
//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char** argv)
{
    // node setup
    ros::init(argc, argv, "dynamixel_client");
    ros::NodeHandle nh;

    // client setting
    dynamixel_client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

    // requset from client to server
    dynamixelCommandMsg("", 1, "Torque_Enable", 1);
    sleep(2);

    dynamixelCommandMsg("", 1, "Goal_Velocity", 5);
    sleep(2);

    dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
    sleep(2);

    dynamixelCommandMsg("", 1, "Torque_Enable", 0);
    sleep(2);


    return 0;
}