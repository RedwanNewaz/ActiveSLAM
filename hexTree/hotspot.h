#ifndef HOTSPOT_H
#define HOTSPOT_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "infotree.h"
#include "dpsolver.h"
#include <iostream>
#include "geometry_msgs/PoseArray.h"
#include "active_slam/obstacle.h"
#include "active_slam/sensor.h"
#include "active_slam/measurement.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sstream>
#include "coverage.h"
#include "datalogger.h"


using namespace Eigen;
class dpsolver;
class infoTree;
class coverage;
class hotspot
{
public:
    hotspot();
    void debugger(std::string ss);
    void findHotspot(float robot[2],float goal[2]);
    void updateMeasurement(float attribute[6]);
    void pathSeqToMes(float reading[6],int n);
private:
    bool enableSimulation;
    ros::Timer timer;
    ros::NodeHandle nh;
    float initialRobotPosition[2],target[2],uav[2];
    int step;
    infoTree *tree;
    dpsolver *dp;
    ros::NodeHandle nh_;
    ros::Publisher traj,debugger_cntrl;
    float HexSamples[7][2];
    ros::ServiceClient robot_client,measurement_client;
    ros::ServiceServer service;
    ros::Subscriber rviz_pose;
    int path_length, previous_direction;
    float backgroundMeasurement,meas_gain;
    bool finishSearch;
    coverage *area_coverage;
    datalogger *datalog;


protected:
    void timerCallback(const ros::TimerEvent& e);
    void simulation();
    void updateParentLocation(float *robot,int candidate);
    void readChildren(float* xx,float* yy,int i,int j,int k,int l);
    void lineIntesection(float *center, float xx[4], float yy[4]);
    Matrix<float,Dynamic,1>  distance(MatrixXf,MatrixXf,int size=6);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr  msg);

    void minMax(float* array,float* max,float* min);
    void sampling_path_publish();
    void optimal_path_publish();
    MatrixXf repeatMat(float *array);
    MatrixXf array2Mat(float array[7][2]);
    bool sensor_reading(active_slam::sensor::Request  &req,
                        active_slam::sensor::Response &res);



};

#endif // HOTSPOT_H
