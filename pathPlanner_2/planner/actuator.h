#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "../header.h"
#include "Dstar.h"
#define PLANNINGSCALE 10.00
#define CLOUDSCALE 6.0
using namespace std;

typedef struct{
    double x,y;
}TODPOSITION;
class display;
class Dstar;
class actuator

{

public:
    actuator();
    void run(TODPOSITION,TODPOSITION);
    void map();
    void printPath();
    void statePublish(vector<double>);
    void pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    void writemap(const nav_msgs::OccupancyGridPtr msg);

private:
     ros::NodeHandle nh_;
     ros::Publisher dronepose_pub,map_modify;
     bool mapExist;
     Dstar *dstar;
     display *viz;
     list<state> mypath;
     ros::Publisher pcl_pub2;
     QMutex mutex;
     struct ocotmap{
         double grid[1000][1000];
         int height,width;
         float originX,originY;
     }map2D;
protected:
     void searchSpace();
     void obstacleMapPublish();



};

#endif // ACTUATOR_H
