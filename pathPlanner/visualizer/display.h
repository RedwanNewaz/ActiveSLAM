#ifndef DISPLAY_H
#define DISPLAY_H
#include "../header.h"
#include <visualization_msgs/Marker.h>
#include "../planner/Dstar.h"

typedef std::vector< std::vector< float > > item;


class display
{
public:
    display();
    void obstacleMap( item collision, bool search=false);
    void pathShow(list<state>);
    void pathViz(std::vector<double> x,std::vector<double> y);
    void mapPublisher(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
private:
    ros::Publisher robot_pub;
    ros::Publisher pcl_pub2,pcl_pub_plan;
    ros::NodeHandle nh_;
    struct track{
        std::vector<double> x,y,z;
    }positionTracker;

    visualization_msgs::Marker robot,track_list,plan;
    uint32_t shape;



};

#endif // DISPLAY_H
