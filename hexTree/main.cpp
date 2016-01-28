#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "hotspot.h"




float robot[2]={0,0};

void goalCallback(const geometry_msgs::PoseStampedConstPtr msg){
     ROS_INFO("goal (%f, %f) ",msg->pose.position.x,msg->pose.position.y);
     float goal[2]={msg->pose.position.x,msg->pose.position.y};
     hotspot HP;
     HP.findHotspot(robot,goal);
 }



int main(int argc, char *argv[])
{
    ros::init(argc,argv,"HotspotSeeking");
    ros::NodeHandle nh_;
    ros::Subscriber rviz_goal;

    //RVIZ DEPENDECIES
    rviz_goal=nh_.subscribe("move_base_simple/goal",10,goalCallback);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
