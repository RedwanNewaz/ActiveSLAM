#ifndef STATEESTIMATION_H
#define STATEESTIMATION_H

#include "../header.h"
#include "active_slam/obstacle.h"
#include "active_slam/measurement.h"
#include "../../hexTree/datalogger.h"
#define nano 1000000000.00
#define T6 1000000.00
#define T3 1000.00
#define mm  1000.00
#define cm 1000.00
#define g 9.81
#define deg 0.0174532925
#define ALT_TH 2.5



class scale;
class datalogger;
using namespace Eigen;
using namespace std;

extern unsigned int ros_header_timestamp_base;
// gets a relative ms-time from a ROS time.
// can only be used to compute time differences, as it has no absolute reference.
inline static int getMS(ros::Time stamp = ros::Time::now())
{
        if(ros_header_timestamp_base == 0)
        {
                ros_header_timestamp_base = stamp.sec;
                std::cout << "set ts base to " << ros_header_timestamp_base << std::endl;
        }
        int mss = (stamp.sec - ros_header_timestamp_base) * 1000 + stamp.nsec/1000000;
		
        if(mss < 0)
                std::cout << "ERROR: negative timestamp..."<< std::endl;
        return mss;
}
//	long rosTS = getMS(lastNavdataReceived.header.stamp);

inline static Eigen::Matrix3d rpy2rod(double roll, double pitch, double yaw){
    Eigen::Matrix3d  mat;

    double sa = sin(yaw);	// a is yaw = psi
    double ca = cos(yaw);
    double sb = sin(roll);	// b is roll = phi
    double cb = cos(roll);
    double sg = sin(pitch);	// g is pitch = theta
    double cg = cos(pitch);

    mat(0,0) = ca*cb;
    mat(0,1) = sa*cb;
    mat(0,2) = -sb;

    mat(1,0) = ca*sb*sg-sa*cg;
    mat(1,1) = sa*sb*sg+ca*cg;
    mat(1,2) = cb*sg;

    mat(2,0) = ca*sb*cg+sa*sg;
    mat(2,1) = sa*sb*cg-ca*sg;
    mat(2,2) = cb*cg;
    return mat;
}

inline Eigen::Vector3d camToworld(Eigen::Vector3d p){
    Eigen::Transform<double,3,Affine>  R;
    R=AngleAxisd(M_PI/2, Vector3d::UnitX())*AngleAxisd(M_PI, Vector3d::UnitY());
    ROS_INFO_STREAM(R.rotation());
    p=R.rotation()*p;
    p(0)=-p(0);//roll
    p(1)=-p(2);//pitch
    p(2)=-p(1);//yaw
    return p;
}

enum SENSOR_DATA{
    NAVDATA,
    IMUDATA,
    SLAMDATA,
    FUSEDATA,
	TUMDATA
};

class stateEstimation
{
public:
    stateEstimation();

    void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
    void slamCb(const geometry_msgs::PoseConstPtr cam);
    void imudataCb(const sensor_msgs::ImuConstPtr msg);


    double battery();
    void printState(SENSOR_DATA option);
    void ukf_feedback(const nav_msgs::OdometryConstPtr Odom_msg);
    std::vector<double> stateDisplay(SENSOR_DATA);


//    Eigen::Vector3d rpy( Matrix3d R);
    Matrix3d Rot3D;

    struct statespace{
        double x,y,z,vx,vy,vz,ax,ay,az,phi,theta,psi,wx,wy,wz,t;
    }navState,slamState,imuState,ukfState,ukfInput;


    struct eigenspace{
        Eigen::Vector3d p,v,a,old_p;
    }eigenstate;

    void predictedState(statespace state, SENSOR_DATA option);
    void dataWrite(SENSOR_DATA option, statespace state);
    vector<double> stateMSG();
	void comparision(vector<double>);

    eigenspace state2eign(statespace state);

private:
    double batteryStatus;
    double VOSCALE,VEL_SCALE,ACC_SCALE,time_step,vel_sum;
    double est_altd;
    double lastYaw;
    int movingState;
    bool VOSTART,initialized_scale;
    uint32_t lastTime;
    QMutex mutex;
    bool zOFFSET;
    double zHeightoffset;
    double last_dot_yaw;
	QString navDir,imuDir,slamDir,fuseDir,tumDir;

    scale *scaleEst;


    ros::Time starttime;
    uint32_t navdataCount;
    ros::Publisher debugger_cntrl;
    ros::Publisher slam_pub;
    ros::Publisher nav_pub;
    ros::Publisher imu_pub;
    ros::NodeHandle nh;
    ros::Timer timer;

    //path planner localization
    ros::ServiceServer robot_srv,calib_srv;
    datalogger *NaVmap,*ImUmap,*SlaMmap,*FUSEDmap,*TUMmap;


protected:
    double timeDiff(ros::Time start);
    void state2navData(statespace state, char* dataType);
    void logfile_Init();
    void resetState();
    void timerCallback(const ros::TimerEvent& e);
    stateEstimation::statespace mirrorTransform(statespace);
    bool computeVoScale();
    bool localization(active_slam::obstacle::Request  &req,
                      active_slam::obstacle::Response &res);

    bool calibration(active_slam::measurement::Request  &req,
                     active_slam::measurement::Response &res);

    void debugger(std::string ss);
    };

#endif // STATEESTIMATION_H
