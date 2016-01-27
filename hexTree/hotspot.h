#ifndef HOTSPOT_H
#define HOTSPOT_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "infotree.h"
#include "dpsolver.h"
#include <iostream>


using namespace Eigen;
class dpsolver;
class infoTree;
class hotspot
{
public:
    hotspot();
    void findHotspot(float robot[2],float goal[2]);
private:
    bool enableSimulation;
    ros::Timer timer;
    ros::NodeHandle nh;
    float initialRobotPosition[2],target[2],uav[2];
    long step;
    infoTree *tree;
    dpsolver *dp;

protected:
    void timerCallback(const ros::TimerEvent& e);
    void simulation();
    void updateParentLocation(float *robot,int candidate);
    void readChildren(float* xx,float* yy,int i,int j,int k,int l);
    void lineIntesection(float *center, float xx[4], float yy[4]);
    Matrix<float,Dynamic,1>  distance(MatrixXf,MatrixXf,int size=6);

};

#endif // HOTSPOT_H
