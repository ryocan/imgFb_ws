#include "softhand_mg400_img_control/imgProc.h"
#include <math.h>

int main(int argc, char** argv)
{
    // ros setup
    ros::init(argc, argv, "roi");

    // input img
    Mat img_input = imread("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/input_images/with obj/1b.jpg");

    cout << "img.cols: " << img_input.cols << endl;
    cout << "img.rows: " << img_input.rows << endl;
    Mat roi_src(img_input, cv::Rect(600, 0, 500, 1024));
    imshow("roi", roi_src);
    imwrite("/home/umelab/imgFb_ws/src/softhand_mg400_img_control/img/input_images/with obj/input_1b.jpg", roi_src);

    waitKey(0);
    return 0;
}