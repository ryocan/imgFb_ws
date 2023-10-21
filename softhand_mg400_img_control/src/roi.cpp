#include "softhand_mg400_img_control/imgProc.h"
#include <math.h>

int main(int argc, char** argv)
{
    // ros setup
    ros::init(argc, argv, "roi");
    ros::NodeHandle nh;

    // input img
    // string input_path;
    // string input_img_num;
    // string character;
    string input_path = "/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/input_images/with obj/pear/poseA";
    string input_img_num = "/151";
    string character = "a";


    //-------------------------------------------------------
    // nh.getParam("input_path", input_path);
    // nh.getParam("input_img_num", input_img_num);
    // nh.getParam("character", character);
    // cout << "input_path-----" << input_path << endl;
    // cout << "input_img_num-----" << input_img_num << endl;
    // cout << "character-----" << character << endl;

    // int test;
    // nh.getParam("test", character);
    // cout << "debug: test-----" << test << endl;
    // //-------------------------------------------------------
    // if (nh.getParam("input_path", input_path)) {
    //     ROS_INFO("Input path: %s", input_path.c_str());
    // } else {
    //     ROS_ERROR("Failed to get parameter");
    //     return 1;
    // }

    Mat img_input_1= imread(input_path + input_img_num + "_a.jpg");
    Mat img_input_2= imread(input_path + input_img_num + "_b.jpg");

    // cout << "img.cols: " << img_input.cols << endl;
    // cout << "img.rows: " << img_input.rows << endl;
    Mat roi_src_1(img_input_1, cv::Rect(600, 0, 500, 1024));
    Mat roi_src_2(img_input_2, cv::Rect(600, 0, 500, 1024));


    imshow("roi_src_1", roi_src_1);
    imshow("roi_src_2", roi_src_2);
    imwrite(input_path + input_img_num + "_roi_a.jpg", roi_src_1);
    imwrite(input_path + input_img_num + "_roi_b.jpg", roi_src_2);

    waitKey(0);
    return 0;
}