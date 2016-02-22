/*SCALE estimation problems
 * VO and IMU has different scale
 * we need to calibrate each of them
 * From the 1meter real world distance
 * TODO make a gui button for calibration
 */


#include "stateestimation.h"
#include "../header.h"
#include <QFile>

 stateEstimation::stateEstimation()
{
    VOSCALE=1;
    scaleEst=new scale();
    //cntrl=new controller();
    resetState();
    starttime= ros::Time::now();
    batteryStatus=0;
    VOSTART =false;
    timer = nh.createTimer(ros::Duration(0.03), &stateEstimation::timerCallback,this);
    debugger_cntrl=nh.advertise<std_msgs::String>("jaistquad/debug",1);

    lastTime=0;
    zOFFSET=initialized_scale=false;
    zHeightoffset=navdataCount=0;

    //path planner localization
    robot_srv = nh.advertiseService("localization",&stateEstimation::localization,this);
    calib_srv = nh.advertiseService("calibration",&stateEstimation::calibration,this);

    //yaw
    lastYaw=time_step=vel_sum=0;
    last_dot_yaw=0;

    //PUBLISHER TOPICS
     slam_pub= nh.advertise<nav_msgs::Odometry>("ORB_SLAM", 5);
     nav_pub= nh.advertise<nav_msgs::Odometry>("navData", 5);
     imu_pub= nh.advertise<nav_msgs::Odometry>("imu", 5);
	 
	 //writing files 
     logfile_Init();

}

 void stateEstimation::predictedState(statespace state, SENSOR_DATA option){

    if(!VOSTART){
        if(initialized_scale)computeVoScale();
        return;
    }

    //compute relative time
    double current=getMS();
    double dt =(current- state.t)/nano;
    dt*=0.03;
    if(dt<0){
        ROS_ERROR("TIME CANT BE NEGATIVE %ld",dt );
        return;}
    

    // fixing altitude
    if(est_altd>0 && ukfState.z!=0 && movingState==4 && !zOFFSET){
        zHeightoffset=est_altd-ukfState.z;
        zOFFSET=true;
    }
    else if(zOFFSET && (movingState==2 ||movingState==8 ||movingState==1) ){
        zOFFSET=false;
        zHeightoffset=0;

    }

    mutex.lock();
    statespace nav2slam;
    switch (option){
        case NAVDATA: 
           
         nav2slam=mirrorTransform(state);
         state.x+=nav2slam.vx*dt+0.5*nav2slam.ax*dt*dt;
         state.y+=nav2slam.vy*dt+0.5*nav2slam.ay*dt*dt;
         state.z+=nav2slam.vz*dt+0.5*nav2slam.az*dt*dt;
         state.vx=nav2slam.vx+nav2slam.ax*dt;
         state.vy=nav2slam.vy+nav2slam.ay*dt;
         state.vz=nav2slam.vz+nav2slam.az*dt;
         ukfInput=state;
         state2navData(ukfInput,"navData");
          // ROS_INFO_STREAM("NAVDATA " <<dt<<"\t"<< state.vx 
          //                           <<"\t"<< state.vy
          //                           <<"\t"<< state.vz
          //   );
         break;
        case IMUDATA: 

         nav2slam=mirrorTransform(state);
         state.x+=nav2slam.vx*dt+0.5*nav2slam.ax*dt*dt;
         state.y+=nav2slam.vy*dt+0.5*nav2slam.ay*dt*dt;
         state.z+=nav2slam.vz*dt+0.5*nav2slam.az*dt*dt;
         state.vx=nav2slam.vx+nav2slam.ax*dt;
         state.vy=nav2slam.vy+nav2slam.ay*dt;
         state.vz=nav2slam.vz+nav2slam.az*dt;
         ukfInput=state;
        state2navData(ukfInput,"imu");
            //       ROS_INFO_STREAM("IMUDATA " <<dt<<"\t"<< state.vx 
            //                         <<"\t"<< state.vy
            //                         <<"\t"<< state.vz
            // );
        break;
        case SLAMDATA:
         state.x+=ukfInput.vx*dt+0.5*ukfInput.ax*dt*dt;
         state.y+=ukfInput.vy*dt+0.5*ukfInput.ay*dt*dt;
         state.z+=ukfInput.vz*dt+0.5*ukfInput.az*dt*dt;
         state.vx=ukfInput.vx+ukfInput.ax*dt;
         state.vy=ukfInput.vy+ukfInput.ay*dt;
         state.vz=ukfInput.vz+ukfInput.az*dt;
         ukfInput=navState=imuState=state;
        state2navData(ukfInput,"slam");
            //       ROS_INFO_STREAM("SLAMDATA " <<dt<<"\t"<< state.vx 
            //                         <<"\t"<< state.vy
            //                         <<"\t"<< state.vz
            // );
        break;
        }
        mutex.unlock();

//    return;
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
     slamState=imuState=ukfState=ukfInput=navState;
 }

 stateEstimation::statespace stateEstimation:: mirrorTransform(statespace state){
    Matrix3d R;
    Eigen::Vector3d
           position(state.x,state.y,state.z),
           velocity(state.vx,state.vy,state.vz),
           acceleration(state.ax,state.ay,state.az),
           Orientaion(state.phi,state.theta,state.psi);
    R << 0, -1, 0,   1, 0, 0,   0, 0, 1;
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
    navState.t=getMS();
    navState.vx=navdataPtr->vx/mm;navState.vy=navdataPtr->vy/mm;navState.vz=navdataPtr->vz/mm;
    navState.ax=navdataPtr->ax*g;navState.ay=navdataPtr->ay*g;navState.az=g-navdataPtr->az*g;
    navState.phi=navdataPtr->rotX*deg;navState.theta=navdataPtr->rotY*deg;navState.psi=navdataPtr->rotZ*deg;

    batteryStatus=navdataPtr->batteryPercent;
    est_altd=navdataPtr->altd/cm;
    movingState=navdataPtr->state;
//    qDebug()<<est_altd;
//    predictedState(navState,NAVDATA);



}

 //change initalized_scale from gui
 bool stateEstimation::calibration(active_slam::measurement::Request  &req,
                   active_slam::measurement::Response &res)
 {
     debugger("calibration started");
     sleep(1);
     return initialized_scale=true;
 }



 void stateEstimation::slamCb(const geometry_msgs::PoseConstPtr cam){
  //  initialized_scale=true;
    Matrix3d R,R1;
    Eigen::Vector3d t(cam->position.x,cam->position.y,cam->position.z),RpY(cam->orientation.x,cam->orientation.y,cam->orientation.z);
    R << 1, 0, 0,   0, 0, -1,   0, 1, 0;
    R1 << 1.0, 0, 0,   0, 0, 1.0,   0, -1.0, 0;
    double dt=slamState.t=getMS();
    starttime= ros::Time::now();
    // Transform the cordinate

    t=VOSCALE*R*t;
    RpY=R1*RpY;

    //calculate the diffierence first
    slamState.vx=(t(0)-slamState.x);slamState.vy=(t(1)-slamState.y);slamState.vz=(t(2)-slamState.z);

    //update the measurement
    slamState.x=t(0);slamState.y=t(1);slamState.z=t(2);
    slamState.vx*=dt;slamState.vy*=dt;slamState.vz*=dt;
    slamState.ax= slamState.vx*dt;slamState.ay=slamState.vy*dt;slamState.az=slamState.vz*dt;
    slamState.phi=RpY(0);slamState.theta=RpY(1);slamState.psi=RpY(2);
    //if(VOSTART)
    //cntrl->v_slam_time_update(getMS());



}
// scale estimation
 bool stateEstimation::computeVoScale(){
    if(!initialized_scale||slamState.z==0){
        resetState();
        return false;
    }


        VOSCALE=50;

        VEL_SCALE=VOSCALE;
        ACC_SCALE=VEL_SCALE*VEL_SCALE/2;

        resetState();
        VOSTART =true;
        initialized_scale=false;
        stringstream ss;
        ss<<"VO SCALE "<<VOSCALE <<" VEL_SCALE "<<VEL_SCALE
            <<" time_step "<<ACC_SCALE;
        debugger(ss.str());
        sleep(1);
        time_step=vel_sum=0;



//    time_step++;
//    vel_sum+=imuState.ax;
//    if(slamState.z==0)return false;
//    static double init_y=slamState.y;
//    static double init_alt=slamState.z;

//       int z_down;
//       if(init_alt<0)
//            z_down=init_alt/slamState.z;
//           else
//       z_down=slamState.z/init_alt;
//       bool result =false;
//          ROS_ERROR("Altitude %f",abs(z_down));

//    if(abs(z_down)>ALT_TH)
//        result=true;

//    //scaleEst->sampling(slamState.x,slamState.y,nav2slam.vx,nav2slam.vy,nav2slam.ax,nav2slam.ay);
//     if(result){
//         VOSCALE=1/abs(slamState.y-init_y);

//         VEL_SCALE=(1/(pow(time_step*0.03,2)*vel_sum));
//         ACC_SCALE=VEL_SCALE*VEL_SCALE/2;

//         resetState();
//         VOSTART =true;
//         initialized_scale=false;
//         stringstream ss;
//         ss<<"VO SCALE "<<VOSCALE <<" VEL_SCALE "<<VEL_SCALE
//             <<" time_step "<<ACC_SCALE;
//         debugger(ss.str());
//         sleep(1);
//         time_step=vel_sum=0;
//     }
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

    imuState.t=getMS();

}

 void stateEstimation::timerCallback(const ros::TimerEvent& e){

     if(slamState.t<=navState.t && slamState.t<=imuState.t)
          predictedState(slamState, SLAMDATA);
     else if(navState.t<=imuState.t)
          predictedState(navState, NAVDATA);
     else
        predictedState(imuState, IMUDATA);


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

stateEstimation::eigenspace stateEstimation::state2eign(statespace state){
    eigenspace eigst;
    eigst.p<<state.x,state.y,state.z;
    eigst.v<<state.vx,state.vy,state.vz;
    eigst.a<<state.ax,state.ay,state.az;
    return eigst;
}



 //SERVICES
bool stateEstimation::localization(active_slam::obstacle::Request  &req,
         active_slam::obstacle::Response &res)
{
    ROS_INFO_STREAM("localization message received");

    res.x=ukfState.x;//robot x
    res.y=ukfState.y;//robot y

    return true;

}

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


       robot.push_back(state.phi);
       robot.push_back(state.theta);
       robot.push_back(state.psi);

       return robot;
  }

 void stateEstimation::ukf_feedback(const nav_msgs::OdometryConstPtr Odom_msg){

  
     double x=Odom_msg->pose.pose.position.x, y=Odom_msg->pose.pose.position.y,
            z=Odom_msg->pose.pose.position.z;
     double vx=Odom_msg->twist.twist.linear.x,
            vy=Odom_msg->twist.twist.linear.y,
            vz=Odom_msg->twist.twist.linear.z;

                if(abs(vx)>2||abs(vy)>2) return;
    mutex.lock();
     ukfState.t=timeDiff(Odom_msg->header.stamp);
     ukfState.x=x;ukfState.y=-y;ukfState.z=-z+zHeightoffset;
     ukfState.vx=vx;ukfState.vy=-vy;ukfState.vz=-vz;
     Eigen::Vector3d slaMori(slamState.phi,slamState.theta,slamState.psi);
     ukfState.phi=slaMori(0);ukfState.theta=slaMori(1);ukfState.psi=slaMori(2);
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
 void stateEstimation::comparision(vector<double>ptam){
		QString name;
		statespace state;	
    ROS_WARN("TUM STATE");	
	for(int i=0;i<2;i++){
		switch(i){
			case 0:name=fuseDir;
				state=ukfState;
				break;
			case 1:name=tumDir;
			state.x=ptam.at(0);state.y=ptam.at(1);state.z=ptam.at(2);state.psi=ptam.at(8);
			state.vx=ptam.at(3);state.vy=ptam.at(4);state.vz=ptam.at(5);
			break;}
			QFile file(name);
			file.open(QIODevice::Append | QIODevice::Text);
			if(file.isOpen()){
			  QTextStream outStream(&file);
				outStream<<
							getMS()<<"\t"<< state.x<<"\t"<<state.y<<"\t"<<state.z<<"\t"<<state.psi<<"\t"<<
							state.vx<<"\t"<<state.vy<<"\t"<<state.vz<<"\n";}
			file.close();
    }

}
 void stateEstimation::dataWrite(SENSOR_DATA option, statespace state){

    float a[10]={getMS(),state.x,state.y,state.z,state.vx,state.vy,state.vz,state.ax,state.ay,state.az};
    switch (option){
        case NAVDATA:      NaVmap   ->dataWrite(a,10); break;
        case IMUDATA:      ImUmap   ->dataWrite(a,10); break;
        case SLAMDATA:     SlaMmap  ->dataWrite(a,10); break;
        case FUSEDATA:     FUSEDmap ->dataWrite(a,10); break;
    }




}

 vector<double>  stateEstimation::stateMSG(){

     vector<double>msg;
     mutex.lock();
    
     //      //    POSITION
     msg.push_back(ukfState.x);
     msg.push_back(ukfState.y);
     msg.push_back(ukfState.z);
     msg.push_back(ukfState.psi);

     msg.push_back(ukfState.vx);
     msg.push_back(ukfState.vy);
     msg.push_back(ukfState.vz);
     double dyaw=(ukfState.psi-lastYaw)/0.03;
     if(lastYaw!=ukfState.psi)
     lastYaw=ukfState.psi;
     msg.push_back(dyaw);

     statespace nav2ukf=mirrorTransform(navState);
     msg.push_back(nav2ukf.ax);
     msg.push_back(nav2ukf.ay);
     msg.push_back(nav2ukf.az);
     double a_yaw=(dyaw-last_dot_yaw)/0.03;
     msg.push_back(a_yaw);
     last_dot_yaw=dyaw;

     msg.push_back(ukfState.phi);
     msg.push_back(ukfState.theta);

     mutex.unlock();
      return msg;

 }


 void stateEstimation::logfile_Init(){

     ROS_WARN("initialization of datalogger");

     NaVmap   =new datalogger;
     ImUmap   =new datalogger;
     SlaMmap  =new datalogger;
     FUSEDmap =new datalogger;
//     TUMmap   =new datalogger;

     string headername[] = {"MS","x","y","z","vx","vy","vz","ax","ay","az"   };
     NaVmap->fileName("Navdata_log");
     ImUmap->fileName("Imudata_log");
     SlaMmap->fileName("VOdata_log");
     FUSEDmap->fileName("SensorFusion_log");
//     TUMmap->fileName("TUMmap");

     sleep(1);

     NaVmap   ->addHeader(headername,10);
     ImUmap   ->addHeader(headername,10);
     SlaMmap  ->addHeader(headername,10);
     FUSEDmap ->addHeader(headername,10);
//     TUMmap   ->addHeader(headername);

 }

 void stateEstimation::debugger(std::string ss){
     std_msgs::String debug_cntr;
     debug_cntr.data=ss;
     debugger_cntrl.publish(debug_cntr);
     ROS_INFO_STREAM(ss);
 }

// double stateEstimation::distance(double x1, double y1, double x2, double y2)
// {
//     return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
// }
