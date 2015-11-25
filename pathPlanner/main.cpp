#include "header.h"

unsigned int ros_header_timestamp_base = 0;
using namespace std;
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"planningServer");
    ROS_INFO("planner vs 2.00");

    mav uav;
    uav.run();
    ros::spin();
    return 0;
}
