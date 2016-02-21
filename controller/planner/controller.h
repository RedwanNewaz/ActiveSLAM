#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "active_slam/pidgain.h"
#define degree 0.0174533
#define UNITSTEP 0.1
#define FREQUENCY 30.00
#define STOP 0.3
#define MAX_VEL 2
#define GARBAGE_MES 1000
// #define red 10.00

#define  MAX_U 1
#define  MIN_U -1
#define  RANGE 0.1
#define rvizScale 1
#include <Eigen/Core>
#include <QString>
#include <QMutex>
#include <QDebug>
#include "geometry_msgs/PoseArray.h"
#include "active_slam/measurement.h"
#include "active_slam/sensor.h"
#include "active_slam/plannertalk.h"

#include "../header.h"
#include "../../hexTree/datalogger.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include <iterator>


inline float min_ele_vec(std::vector<float> v){
  std::vector<float>::iterator result = std::min_element(v.begin(), v.end());
  return result[0];
}


//prediction model coefficient
static const double c1=0.01;
 static const double c2=0.3;
 static const double c3=0.12;
 static const double c4=0.418;
 static const double c5=0.436;
 static const double c6=1.57;
 static const double c7=0.8;
 static const double c8=0.6;


//PD default parameters
static const double KpX=0.11;         //0.5727  0.11  1.27;
    static const double KdX=0.11;		//1.538   0.22  0.2588
static const double KpY=0.11; 		//0.51 0.2144 0.4685 0.2144;
    static const double KdY=0.11;		//0.323 0.4225 1.314 0.4225;
static const double KpZ=0.6;
    static const double KdZ=0.1;
static const double KpS=0.05;
    static const double KdS=0.0;

class datalogger;
using namespace Eigen;
class controller
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    controller();
    // The main callback to calculate u
    void control(vector<double> msg);
    void stateUpdate(vector<double> msg);
    void trajCallback(const geometry_msgs::PoseArrayConstPtr msg);
    void executingTraj();
    void debugger(std::string);
    void readGain();
    struct tr{
        vector<double>x,y;
        int index;
    }traj;


private:

    vector<double> setpoint;
    ros::Timer timer;
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::ServiceServer service,attribute,threshold,mode;
    bool nocomm_vslam,land_cmd,stablizing;
    int count;
    int resetController;
    double error;
    double vslam_count;

    //QT
    QString fileName;
    QMutex mutex;
	
	//circle
	bool EnableCircle;
	float circleCount;
	ros::Timer Circletimer;

    //traj and p2p
    bool plan;
    bool EnableTraj, EnableP2P,state_update;


    //debugger
    ros::Publisher debugger_cntrl,land_pub;
    ros::Subscriber xbee,camPose_sub;

    std_msgs::String debug_cntr;

    ros::ServiceClient obs_srv;

    //XBEE READING
    int obstacleStatus;
    float lightSensor[3];


    enum target{
        RECEVIED,
        EXECUTING,
        SUCCESS,
        IDLE,
        INTIALIZING,
        FAILED
    }controller_Status;


    struct ControlCommand
    {
        inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
        inline ControlCommand(double roll, double pitch, double yaw, double gaz)
        {
            this->roll = roll;
            this->pitch = pitch;
            this->yaw = yaw;
            this->gaz = gaz;
        }
        double yaw, roll, pitch, gaz;
    };
    struct orientation{
        double roll,pitch,yaw;
    }ori;

//controller variable
            Eigen::Matrix4d kp,kd;
            Eigen::Vector4d X,Vel,X_error,X_dot_error,U,Nu;
            std::vector<double>velocity,acceleration,input;
            double V_target,V_dot_target;
            bool inputApply;
            double gain[8];

// Source seeking variable
            struct seek{
                int length,index;
                float intensity[6], threshold;
                bool Obstacle;
            }path;

//datalogger
            datalogger *log,*cntrl_per;
            int global_index;
            vector<float> converage;

protected:

    void observer();
    void cmdQueuePub();
    void cmdPublish(ControlCommand);
    bool goalConverage();
    double prediction(int);
//    QMutex mutex;
    void dataWrite();
    void updateGain();
    void wrtieGain();
    void motionType(int);
    void modeSelction(int);
    void HoveringMode();

    void SendMeasurementPacket();
    void updateMeasurement(bool fake=false);

    //PID parameters
    double i_term[3];



    //service
    bool gainchange(active_slam::pidgain::Request  &req,
             active_slam::pidgain::Response &res);
    bool measurements_attribute(active_slam::measurement::Request  &req,
                                active_slam::measurement::Response &res);
    bool measurements_threshold(active_slam::measurement::Request  &req,
                                active_slam::measurement::Response &res);
    void xbeeRead(const geometry_msgs::QuaternionConstPtr);
    bool talk(active_slam::plannertalk::Request  &req,
             active_slam::plannertalk::Response &res);
    void camCallback(const geometry_msgs::PoseConstPtr cam);



    bool compute_X_error();


//    controller design

    Eigen::Vector4d EivenVector4dNormalize(Eigen::Vector4d vec);
    void run(const ros::TimerEvent& e);
    Eigen::Vector4d Ar_drone_input();
};

#endif // CONTROLLER_H
