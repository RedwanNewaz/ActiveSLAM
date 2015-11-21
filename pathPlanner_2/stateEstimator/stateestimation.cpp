#include "stateestimation.h"
#include "../header.h"
#include <QFile>

 stateEstimation::stateEstimation()
{

    resetState();
    starttime= ros::Time::now();
    batteryStatus=0;
    VOSTART =false;
    timer = nh.createTimer(ros::Duration(0.5), &stateEstimation::timerCallback,this);
    lastTime=0;


    //PUBLISHER TOPICS
     slam_pub= nh.advertise<nav_msgs::Odometry>("ORB_SLAM", 5);
     nav_pub= nh.advertise<nav_msgs::Odometry>("navData", 5);
     imu_pub= nh.advertise<nav_msgs::Odometry>("imu", 5);

}

 void stateEstimation::predictedState(statespace state, SENSOR_DATA option){
    double t =state.t;
    mutex.lock();
    //POSITION PREDICT
    state.x+=state.vx*t+0.5*state.ax*t*t;
    state.y+=state.vy*t+0.5*state.ay*t*t;
    state.z+=state.vz*t+0.5*state.az*t*t;

    //VELOCITY PREDICT
    state.vx=state.vx+state.ax*t;
    state.vy=state.vy+state.ay*t;
    double vz=state.vz+state.az*t;
    if (vz<=5)state.vz=vz;

    mutex.unlock();
    if(!VOSTART) return;

    switch (option){
        case NAVDATA: navState=state;
             state2navData(mirrorTransform(navState),"navData");
             break;
        case IMUDATA: imuState=state;
             state2navData(mirrorTransform(imuState),"imu");
             break;
        case SLAMDATA:slamState=state;
             state2navData(slamState,"slam");
             break;
    }

    switch (option){
        case NAVDATA: dataWrite(option,mirrorTransform(state));break;
        case IMUDATA: dataWrite(option,mirrorTransform(state));break;
        case SLAMDATA:dataWrite(option,state);break;
    }

}

 void stateEstimation::resetState(){
     navState.t=navState.x=navState.y=navState.z=
     navState.vx=navState.vy=navState.vz=
     navState.ax=navState.ay=navState.az=
     navState.phi=navState.theta=navState.psi=
     navState.wx=navState.wy=navState.wz=0;
     slamState=imuState=ukfState=navState;
 }

 stateEstimation::statespace stateEstimation:: mirrorTransform(statespace state){
    Matrix3d R;
    Eigen::Vector3d
           position(state.x,state.y,state.z),
           velocity(state.vx,state.vy,state.vz),
           acceleration(state.ax,state.ay,state.az),
           Orientaion(state.phi,state.theta,state.psi);
    R << 0, -1, 0,   -1, 0, 0,   0, 0, 1;
    Eigen::Vector3d
           Tp(R*position),
           Tv(R*velocity),
           Ta(R*acceleration),
           TO(R*Orientaion);
    //UPDATE and return state
    statespace stateFeedback{Tp(0),Tp(1),Tp(2),
                             Tv(0),Tv(1),Tv(2),
                             Ta(0),Ta(1),Ta(2),
                             TO(0),TO(1),TO(2)
                             };
    return stateFeedback;

}

 void stateEstimation::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr){
    navState.t=timeDiff(navdataPtr->header.stamp);
    navState.vx=navdataPtr->vx/mm;navState.vy=navdataPtr->vy/mm;navState.vz=navdataPtr->vz/mm;
    navState.ax=navdataPtr->ax*g;navState.ay=navdataPtr->ay*g;navState.az=g-navdataPtr->az*g;
    navState.phi=navdataPtr->rotX*deg;navState.theta=navdataPtr->rotY*deg;navState.psi=navdataPtr->rotZ*deg;

    batteryStatus=navdataPtr->batteryPercent;
    est_altd=navdataPtr->altd/cm;
    movingState=navdataPtr->state;

    predictedState(navState,NAVDATA);



}

 void stateEstimation::slamCb(const geometry_msgs::PoseConstPtr cam){
    if(!VOSTART){
        resetState();
        VOSTART =true;
    }
    Matrix3d R;
    Eigen::Vector3d t(cam->position.x,cam->position.y,cam->position.z),RpY(cam->orientation.x,cam->orientation.y,cam->orientation.z);
    R << 1, 0, 0,   0, 0, -1,   0, 1, 0;
    double dt=slamState.t=timeDiff(starttime);
    starttime= ros::Time::now();
    // Transform the cordinate

    t=R*t;
    RpY=R*RpY;
    //calculate the diffierence first
    slamState.vx=(t(0)-slamState.x);slamState.vy=(t(1)-slamState.y);slamState.vz=(t(2)-slamState.z);

    //update the measurement
    slamState.x=t(0);slamState.y=t(1);slamState.z=t(2);
    slamState.vx*=dt;slamState.vy*=dt;slamState.vz*=dt;
    slamState.ax= slamState.vx*dt;slamState.ay=slamState.vy*dt;slamState.az=slamState.vz*dt;
    slamState.phi=RpY(0);slamState.theta=RpY(1);slamState.psi=RpY(2);

    predictedState(slamState,SLAMDATA);

}

 void stateEstimation::imudataCb(const sensor_msgs::ImuConstPtr msg){
    geometry_msgs::Quaternion orientation=msg->orientation;
    geometry_msgs::Vector3 angular_velocity=msg->angular_velocity;
    geometry_msgs::Vector3 linear_acceleration=msg->linear_acceleration;

    double roll, pitch, yaw;
    tf::Quaternion q(orientation.x,orientation.y,orientation.z,orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    imuState.phi=roll;imuState.theta=pitch;imuState.psi=yaw;
    imuState.vx=imuState.vy=imuState.vz=0;
    imuState.ax=linear_acceleration.x;imuState.ay=linear_acceleration.y;imuState.az=g-linear_acceleration.z;
    imuState.wx=angular_velocity.x;imuState.wy=angular_velocity.y;imuState.wz=angular_velocity.z;

    imuState.t=timeDiff(msg->header.stamp);

//    imuState=mirrorTransform(imuState);
    predictedState(imuState,IMUDATA);


}

 void stateEstimation::timerCallback(const ros::TimerEvent& e){
    if((getMS()- lastTime)>500 && VOSTART){
        state2navData(navState,"navData");
        sleep(0.010);
        state2navData(imuState,"imu");
        sleep(0.010);
        state2navData(slamState,"imu");
    }
 }

//supporting functions
 double stateEstimation::timeDiff( ros::Time start){
    ros::Time time= ros::Time::now();
    return (time.nsec-start.nsec)/nano;
}

 double stateEstimation::battery(){
    return batteryStatus;
}

 void stateEstimation::state2navData(statespace state, char* dataType){
        nav_msgs::Odometry  odom;
        geometry_msgs::Pose robot_pose;
        odom.header.frame_id="slam";
        odom.header.stamp=ros::Time::now();

//    position update
        odom.pose.pose.position.x=state.x;
        odom.pose.pose.position.y=state.y;
        odom.pose.pose.position.z=state.z;

        robot_pose.position=odom.pose.pose.position;
//    orientation update


//    convert euler to quaternion
        tf::Quaternion q(state.psi,state.theta,state.phi);
        odom.pose.pose.orientation.x=q.getX();
        odom.pose.pose.orientation.y=q.getY();
        odom.pose.pose.orientation.z=q.getZ();
        odom.pose.pose.orientation.w=q.getW();

        robot_pose.orientation=odom.pose.pose.orientation;


        odom.twist.twist.linear.x=state.vx;
        odom.twist.twist.linear.y=state.vy;
        odom.twist.twist.linear.z=state.vz;

        odom.twist.twist.angular.x=state.wx;
        odom.twist.twist.angular.y=state.wy;
        odom.twist.twist.angular.z=state.wz;


        if(dataType=="slam")
            slam_pub.publish(odom);
        else if(dataType=="navData")
            nav_pub.publish(odom);
        else if(dataType=="imu")
            imu_pub.publish(odom);

        lastTime=getMS();

}


 //Helper
 Eigen::Vector3d stateEstimation::rpy( Matrix3d R){
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


 //SERVICES
 std::vector<double> stateEstimation:: stateDisplay(SENSOR_DATA option){
       statespace state;
       std::vector<double> robot;
       switch (option){
           case NAVDATA: state=navState;break;
           case IMUDATA: state=imuState;break;
           case SLAMDATA:state=slamState;break;
           case FUSEDATA:state=ukfState;break;
       }

       robot.push_back(state.x);
       robot.push_back(state.y);
       robot.push_back(state.z);
       return robot;
  }

 void stateEstimation::ukf_feedback(const nav_msgs::OdometryConstPtr Odom_msg){

     mutex.lock();
     double x=Odom_msg->pose.pose.position.x, y=Odom_msg->pose.pose.position.y,
            z=Odom_msg->pose.pose.position.z;
     double qW=Odom_msg->pose.pose.orientation.w,
            qX=Odom_msg->pose.pose.orientation.x,
            qY=Odom_msg->pose.pose.orientation.y,
            qZ=Odom_msg->pose.pose.orientation.z;
     Eigen::Vector3d po(x,y,z),ori;
     Eigen::Quaternion<double>EgNq(qW,qX,qY,qZ);
     Matrix3d mat(EgNq);
     ori=rpy(mat);
     po=po*SCALEFACTOR;
     ukfState.t=timeDiff(Odom_msg->header.stamp);
     ukfState.x=po(0);ukfState.y=-po(1);ukfState.z=-po(2)+1;
     ukfState.phi=ori(0);ukfState.theta=-ori(1);ukfState.psi=-ori(2);
     mutex.unlock();
//     dataWrite(FUSEDATA,ukfState);

 }

 void stateEstimation::printState(SENSOR_DATA option){
    switch(option){
    case NAVDATA: cout<<"navdata\t"<<navState.t<<"\n"<<
                  "position\t"<<navState.x	<< "\t" << navState.y << "\t" << navState.z <<"\n"<<
                  "velocity\t"<<navState.vx	<< "\t" << navState.vy << "\t" << navState.vz <<"\n"<<
                  "aceleration\t"<<navState.ax	<< "\t" << navState.ay << "\t" << navState.az <<"\n"<<
                  "orienataion\t"<<navState.phi	<< "\t" << navState.theta << "\t" << navState.psi <<"\n\n";break;

    case SLAMDATA: cout<<"slamState\t"<<slamState.t<<"\n"<<
                  "position\t"<<slamState.x	<< "\t" << slamState.y << "\t" << slamState.z <<"\n"<<
                  "velocity\t"<<slamState.vx	<< "\t" << slamState.vy << "\t" << slamState.vz <<"\n"<<
                  "aceleration\t"<<slamState.ax	<< "\t" << slamState.ay << "\t" << slamState.az <<"\n"<<
                  "orienataion\t"<<slamState.phi	<< "\t" << slamState.theta << "\t" << slamState.psi <<"\n\n";break;

    case IMUDATA: cout<<"imuState\t"<<imuState.t<<"\n"<<
                  "position\t"<<imuState.x	<< "\t" << imuState.y << "\t" << imuState.z <<"\n"<<
                  "velocity\t"<<imuState.vx	<< "\t" << imuState.vy << "\t" << imuState.vz <<"\n"<<
                  "aceleration\t"<<imuState.ax	<< "\t" << imuState.ay << "\t" << imuState.az <<"\n"<<
                  "orienataion\t"<<imuState.phi	<< "\t" << imuState.theta << "\t" << imuState.psi <<"\n\n";break;
    }

}

 void stateEstimation::dataWrite(SENSOR_DATA option, statespace state){
    char *name;
      switch (option){
          case NAVDATA:name="/home/redwan/Desktop/data/NaVmap.txt";break;
          case IMUDATA:name="/home/redwan/Desktop/data/ImUmap.txt";break;
          case SLAMDATA:name="/home/redwan/Desktop/data/SlaMmap.txt";break;
          case FUSEDATA:name="/home/redwan/Desktop/data/FUSEDmap.txt";break;
      }

        QFile file(name);
                    file.open(QIODevice::Append | QIODevice::Text);

                    if(file.isOpen()){
                          QTextStream outStream(&file);
                            outStream<<
                                        getMS()<<"\t"<< state.x<<"\t"<<state.y<<"\t"<<state.z<<"\t"<<
                                        state.phi<<"\t"<<state.theta<<"\t"<<state.psi<<"\n";

                    }
                    file.close();


}

 vector<double>  stateEstimation::stateMSG(){

     vector<double>msg;
     mutex.lock();
     //    POSITION
     msg.push_back(ukfState.x);
     msg.push_back(ukfState.y);
     msg.push_back(ukfState.z);
     //    VELOCITY
     msg.push_back(ukfState.vx);
     msg.push_back(ukfState.vy);
     msg.push_back(ukfState.vz);
     //    ORIENTATION
     msg.push_back(ukfState.phi);
     msg.push_back(ukfState.theta);
     msg.push_back(ukfState.psi);
     double dyaw=(ukfState.psi-lastYaw)*ukfState.t;
     msg.push_back(dyaw);
     //    UTILITIES
     msg.push_back(batteryStatus);
     msg.push_back(movingState);
     lastYaw=ukfState.psi;

     mutex.unlock();
      return msg;

 }
