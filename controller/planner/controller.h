#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "active_slam/pidgain.h"
#define degree 0.0174533
#define UNITSTEP 0.15
#define FREQUENCY 30.00
#define STOP 0.35
#define MAX_VEL 2
// #define red 10.00

#define  MAX_U 1
#define  MIN_U -1
#define  RANGE 0.1
#define rvizScale 1
#include <Eigen/Core>
#include <QString>
#include <QMutex>
#include "geometry_msgs/PoseArray.h"

#include "../header.h"

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


using namespace Eigen;
class controller
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    controller();
    // The main callback to calculate u
    void obstacleChannel(double,double);
    void control(vector<double> msg);
    void stateUpdate(vector<double> msg);
    bool controllerStatus();
	void circularMotion(const ros::TimerEvent& e);
    void trajCallback(const geometry_msgs::PoseArrayConstPtr msg);
    void executingTraj();
    void debugger(std::string);
    bool obs_state(int *);
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
    ros::ServiceServer service;
    bool testingMode;
    int count;
    int resetController;

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
    ros::Publisher debugger_cntrl;

    std_msgs::String debug_cntr;

    ros::ServiceClient obs_srv;




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



protected:

    void observer();
    void cmdQueuePub();
    void cmdPublish(ControlCommand);
    bool goalConverage();
    void test();
    double prediction(int);
//    QMutex mutex;
    void dataWrite();
    void updateGain();
    void wrtieGain();
    void motionType(int);
    void squarBox();

    //PID parameters
    double i_term[3];



    //service
    bool gainchange(active_slam::pidgain::Request  &req,
             active_slam::pidgain::Response &res);



    bool compute_X_error();


//    controller design

    Eigen::Vector4d EivenVector4dNormalize(Eigen::Vector4d vec);
    void run(const ros::TimerEvent& e);
    Eigen::Vector4d Ar_drone_input();
};

#endif // CONTROLLER_H
