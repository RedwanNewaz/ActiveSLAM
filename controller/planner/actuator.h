#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "../header.h"
#include "Dstar.h"
#define PLANNINGSCALE 10.00
using namespace std;

using namespace Eigen;

inline  Eigen::Vector3d rpy( Matrix3d R){
   double alpha,beta,gamma,pi=M_PI;
   alpha=beta=gamma=0;

   beta = atan2(-R(2,0), sqrt(pow(R(0,0),2) + pow(R(1,0),2)));
   if (beta==pi/2){
     alpha = 0;
     gamma = atan2(R(0,1), R(1,1));}
   else if(beta== -pi/2){
     alpha = 0;
     gamma = -atan2(R(0,1), R(1,1));}
   else{
     alpha = atan2(R(1,0)/cos(beta), R(0,0)/cos(beta));
     gamma = atan2(R(2,1)/cos(beta), R(2,2)/cos(beta));
   }

   Eigen::Vector3d angle(gamma,beta,alpha);
   return angle;

}
inline Eigen::Quaterniond euler2Quaternion(double roll,
                                           double pitch,
                                           double yaw)
{
Eigen::AngleAxisd rollAngle(roll,Eigen::Vector3d::UnitX());
Eigen::AngleAxisd pitchAngle(pitch,Eigen::Vector3d::UnitY());
Eigen::AngleAxisd yawAngle(yaw,Eigen::Vector3d::UnitZ());

Eigen::Quaterniond q = rollAngle*pitchAngle*yawAngle;

return q.normalized();
}

typedef struct{
    double x,y;
}TODPOSITION;

class Dstar;
class actuator

{

public:
    actuator();
    list<state> run(TODPOSITION,TODPOSITION);
    void map();
    void printPath();
    void statePublish(vector<double>);
    void trackerState(vector<double>,int);
    QString trackerFile;

private:
     ros::NodeHandle nh_;
     ros::Publisher dronepose_pub;
     bool mapExist;
     Dstar *dstar;
     list<state> mypath;
     typedef struct{
         double kp,ki,kd;
         double setPoint,error;
     }PID;
protected:
     void searchSpace();
     void PIDcontroller();



};

#endif // ACTUATOR_H
