#ifndef MAV_H
#define MAV_H

#include "header.h"
#include "planner/octomap_search.h"
#include "active_slam/plannertalk.h"



class octomap_search;
class display;
class actuator;


class mav
{

public:
     mav();
    ~mav();

     display *visualize;
     actuator *plan;
     octomap_search *map_search;
     void run();


     void pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
     void takerCallback(const std_msgs::String &msg);

     //OCTOMAP
     void projectedmapCallback(const nav_msgs::OccupancyGridPtr);
     void goalCallback(const geometry_msgs::PoseStampedConstPtr);
     void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr );


private:

     QMutex mutex;
     ros::NodeHandle nh_;
     std::vector<double> roboVect;
     bool p2pNav,ScaleFix;


     //OCTOMAP
     ros::Subscriber projectedmap_sub;

     ros::Subscriber orbTraker;
     ros::Subscriber cloud_sub;


     ros::Subscriber rviz_goal;
     ros::Subscriber rviz_pose;
     ros::Timer timer;
     ros::Publisher planner_talk;
     ros::ServiceServer service,srv_obs;



protected:
     void timerCallback(const ros::TimerEvent& e);
     bool talk(active_slam::plannertalk::Request  &req,
              active_slam::plannertalk::Response &res);

     bool collision(active_slam::obstacle::Request  &req,
                    active_slam::obstacle::Response &res);
};

#endif // MAV_H
