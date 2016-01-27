#ifndef DISPLAY_H
#define DISPLAY_H
#define VISFACTOR 1
#include <ros/ros.h>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
#include <visualization_msgs/Marker.h>

class display
{
public:
    display();
    void hexSample(float SamplePoits[7][2],float localPathIndex[6]);
    void optimalPath(float *x,float*y,int count);

private:
    ros::Publisher robot_pub;
    ros::Publisher pcl_pub2;

    ros::NodeHandle nh_;
    struct track{
        std::vector<double> x,y,z;
    }positionTracker;

    visualization_msgs::Marker robot,track_list,plan,dpoptimal;
    uint32_t shape;

    double robo_x,robo_y,robo_z;
    int hexID;



};

#endif // DISPLAY_H
