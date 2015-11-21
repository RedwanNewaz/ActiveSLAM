#include "header.h"

unsigned int ros_header_timestamp_base = 0;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"ActiveMotion");


    mav uav;
    uav.run();
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
