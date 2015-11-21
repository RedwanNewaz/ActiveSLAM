#include "mav.h"
#include "ros/ros.h"

#include <cstdlib>
#include <iostream>
#include <ctime>
#include <QStringList>

using namespace std;

mav::mav()
{
    stateSpace =new stateEstimation();
    visualize=new display();
    plan =new actuator();
    Cntrl=new controller();
    timer = nh_.createTimer(ros::Duration(0.25), &mav::timerCallback,this);
    controlPose=false;

}

mav::~mav()
{

}



void mav::run(){
        ROS_INFO("MOTION SUBSCRIBTION ENABLE");
        sleep(1);
        Cntrl->debugger("controller started");
        Cntrl->readGain();


        //SENSOR_FUSION DEPENDENCIES
        navdata_sub	   = nh_.subscribe(nh_.resolveName("ardrone/navdata"),50, &mav::navdataCb, this);
        imu_sub    = nh_.subscribe("/ardrone/imu",10, &mav::imudataCb, this);
        ukf_sub	   = nh_.subscribe("sensor/fusion",10, &mav::ukf_localization, this);

        //ORB_SLAM DEPENDECIES
        orbTraker=nh_.subscribe("ORB_SLAM/Debug",10,&mav::takerCallback, this);
//        cloud_sub=nh_.subscribe("/ORB_SLAM/PointCloud_raw",10,&mav::pointcloudCallback, this);
        camPose_sub=nh_.subscribe("/slam/camera",10,&mav::camCallback, this);
        //dronepose_sub=nh_.subscribe(nh_.resolveName("ardrone/predictedPose"),10,&mav::tumSatecallback, this);
        traj_sub=nh_.subscribe("traj",5,&mav::trajCallback,this);


}

void mav::timerCallback(const ros::TimerEvent& e){
     mutex.lock();
          visualize->ukf_transformer(stateSpace->stateDisplay(FUSEDATA));
          int obs[2];
          if(Cntrl->obs_state(obs))
              visualize->obstacle_boundary(obs);
     mutex.unlock();

//     if(!controlPose) return;
//     mutex.lock();
//     vector<double>msg;
//     for(int i=0;i<8;i++)
//         msg.push_back(i);
//      Cntrl->stateUpdate(stateSpace->stateMSG());
//     mutex.unlock();


 }

        //SENSOR_FUSION DEPENDENCIES
void mav::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
//    navdataCb
    mutex.lock();
    stateSpace->navdataCb(navdataPtr);
//    stateSpace->printState(SLAMDATA);
    mutex.unlock();


}

void mav::camCallback(const geometry_msgs::PoseConstPtr cam){
//    slamCb
     mutex.lock();
     stateSpace->slamCb(cam);
//     stateSpace->printState(IMUDATA);
     mutex.unlock();
}

void mav::imudataCb(const sensor_msgs::ImuConstPtr msg){
//    imudataCb
     mutex.lock();
     stateSpace->imudataCb(msg);
//     stateSpace->printState(NAVDATA);
     mutex.unlock();


}

void mav::ukf_localization(const nav_msgs::OdometryConstPtr Odom_msg)
{
//    ukf_transformer
    // qDebug()<<"ukf";
     mutex.lock();
        //ROS_ERROR("UKF STATE");
        stateSpace->ukf_feedback(Odom_msg);
     mutex.unlock();
//        stateMSG()
    mutex.lock();
    // if(controlPose)
         Cntrl->stateUpdate(stateSpace->stateMSG());
    mutex.unlock();


}
        //ORB_SLAM DEPENDECIES
void mav::pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
//        mapPublisher

}

void mav::takerCallback(const std_msgs::String &msg){
    string s;
    std::stringstream ss;
    ss<<msg;
    QStringList f;
        while (getline(ss, s, '\n')){
            QString str = QString::fromUtf8(s.c_str());
            f<<str;
        }
//        qDebug()<<f[2].toInt();

}



void mav::goalCallback(const geometry_msgs::PoseStampedConstPtr msg){
//    QPointF goal_rviz(msg->pose.position.x,msg->pose.position.y);
    mutex.lock();
    vector<double>p;
    p.push_back(msg->pose.position.x/(rvizScale*VISFACTOR));
    p.push_back(msg->pose.position.y/(rvizScale*VISFACTOR));
    p.push_back(1.5);
    p.push_back(0.0);
    Cntrl->control(p);
    controlPose=true;

//        visualize->pathShow(plan->run(TODPOSITION{stateSpace->stateDisplay(FUSEDATA).at(0),stateSpace->stateDisplay(FUSEDATA).at(1)},
//                            TODPOSITION{p.at(0),p.at(1)}));
    mutex.unlock();

}

void mav::tumSatecallback(const tum_ardrone::filter_stateConstPtr msg){
	
	mutex.lock();
     stateDis.clear();
     
     stateDis.push_back(msg->x);
     stateDis.push_back(msg->y);
     stateDis.push_back(msg->z);
     stateDis.push_back(msg->dx);
     stateDis.push_back(msg->dy);
     stateDis.push_back(msg->dz);
     stateDis.push_back(msg->pitch*deg);
     stateDis.push_back(msg->roll*deg);
     stateDis.push_back(-msg->yaw*deg);
	 stateSpace->comparision(stateDis);
	mutex.unlock();



//     stateDis.push_back(msg->roll*deg);
//     stateDis.push_back(msg->pitch*deg);
//     stateDis.push_back(msg->yaw*deg);



 }

//trajecotry

void mav::trajCallback(const geometry_msgs::PoseArrayConstPtr msg){

    Cntrl->trajCallback(msg);

}
