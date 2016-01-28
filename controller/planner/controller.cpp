/*
 * 1) three types of path can be controlled : square,p2p and traj
 * 2) controller frequecy is 30 HZ
 * 3) For controlling XYZ we use PD controller and for Yaw only P controller
 * 4) Prediction based on controller gain is not implemented yet.
 * 5) DATA LOG: Time stamp, position and control command
 * 6) PID GAIN can be recorded to txt file
 */


#include "controller.h"
#include <QFile>
#include <QStringList>
#include "active_slam/obstacle.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Empty.h"
#include "active_slam/sensor.h"




controller::controller()
{
    controller_Status=IDLE;
    plan =false;
    ori.roll=ori.pitch=ori.yaw=0;
    vel_pub	   = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"),1);
    debugger_cntrl=nh.advertise<std_msgs::String>("jaistquad/debug",1);
    service = nh.advertiseService("pid_gains",&controller::gainchange,this);
    obs_srv=nh.serviceClient<active_slam::sensor>("xbee/measurements");
    xbee	   = nh.subscribe("xbeeReading",50, &controller::xbeeRead, this);
    land_pub	   = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/land"),1);







//  controller intialize
     count=obstacleStatus=resetController=traj.index==0;
     for(int i=0;i<4;i++)
         input.push_back(0);
     timer = nh.createTimer(ros::Duration(0.03), &controller::run,this);

//  select motion: type 1/p2p 2/square 3/circular 4/traj follower
    motionType(4);

    X_dot_error<<-0.1,-0.1,-0.1,-0.1;
    U<<0,0,0,0;
    X<<0,0,0,0;
    Vel<<0,0,0,0;
    inputApply=false;
    for (int i=0;i<4;i++){
        acceleration.push_back(0);
        velocity.push_back(0);

    }

    fileName="/home/redwan/Desktop/data/stepRespone.txt";
    QString original="/home/redwan/Desktop/data/stepResponse_";
    for(int i=1;QFile(fileName).exists();i++){
        fileName=original+QString::number(i)+".txt";
    }
    qDebug()<<fileName;
}


void controller::xbeeRead(const geometry_msgs::QuaternionConstPtr msg){
    lightSensor[0]=msg->x;
    lightSensor[1]=msg->y;
    lightSensor[2]=msg->z;
    obstacleStatus=msg->w;
}

//user input
bool controller::gainchange(active_slam::pidgain::Request  &req,
         active_slam::pidgain::Response &res)
{
    res.result=true;
    switch(req.id){
    case 1:gain[0]=req.P;gain[1]=req.D;debugger("roll gain updated");break;
    case 2:gain[2]=req.P;gain[3]=req.D;debugger("pitch gain updated");break;
    case 3:gain[4]=req.P;gain[5]=req.D;debugger("altd gain updated");break;
    case 4:gain[6]=req.P;gain[7]=req.D;debugger("yaw gain updated");break;
    case 5:wrtieGain();break;
    case 6:motionType(req.P);
    }
    sleep(1);
    if(req.id<=4)
    updateGain();
    return true;
}

void controller::motionType(int i){
    EnableTraj=EnableP2P=testingMode=EnableCircle=false;
    switch (i){
        case 1:EnableP2P=true;   debugger("p2p motion selected");   break;
        case 2:EnableTraj=true;squarBox(); debugger("Square motion selected");   break;
        case 3:EnableCircle=true;debugger("Circular motion selected");   break;
        case 4:EnableTraj=true;  debugger("Traj Follower motion selected");   break;
    }
}

void controller::squarBox(){
    traj.x.clear();traj.y.clear();traj.index=0;

    //    goto (0,0)
        traj.x.push_back(0);
        traj.y.push_back(0);

//    goto (0,1)
    traj.x.push_back(0);
    traj.y.push_back(1.5);
    //    goto (1,1)
        traj.x.push_back(1.5);
        traj.y.push_back(1.5);
        //    goto (1,0)
            traj.x.push_back(1.5);
            traj.y.push_back(0);
            //    goto (0,0)
                traj.x.push_back(0);
                traj.y.push_back(0);

                mutex.lock();
                   controller_Status=RECEVIED;
                   executingTraj();
               mutex.unlock();
}

void controller::trajCallback(const geometry_msgs::PoseArrayConstPtr msg){
//     //reseting store trajectory

    active_slam::sensor lightIntensity;
    for(int i=0;i<3;i++)
    lightIntensity.request.reading[i]=i+1;
    lightIntensity.request.count=3;
    if(obs_srv.call(lightIntensity))
        ROS_WARN("Data send");
    //simulation
    sleep(1);

    return;


//     //updating new trajectory
     plan=true;
     traj.x.clear();traj.y.clear();traj.index=0;
    foreach (geometry_msgs::Pose pose,msg->poses){
      ROS_INFO("traj received (%lf, %lf) ", pose.position.x ,pose.position.y);
      traj.x.push_back(pose.position.x);
      traj.y.push_back(pose.position.y);

    }

   std::stringstream debug_cntrle;
   if(EnableP2P){
        foreach (geometry_msgs::Pose pose,msg->poses)
        debug_cntrle<<"approaching to ("<< pose.position.x<<", "<< pose.position.y<<")";
        debugger(debug_cntrle.str());
        controller_Status=RECEVIED;
        executingTraj();
   }
   else if(EnableTraj){
//
    mutex.lock();
       controller_Status=RECEVIED;
       executingTraj();
       debug_cntrle<<traj.x.size();
       debugger("new traj received "+debug_cntrle.str());
    mutex.unlock();
   }


}


void controller::executingTraj(){


             ROS_INFO_STREAM(" remain traj length "<< traj.x.size()-traj.index);
             vector<double> traj_msg;
             traj_msg.push_back(traj.x.at(traj.index));
             traj_msg.push_back(traj.y.at(traj.index));
             traj_msg.push_back(1.2);
             traj_msg.push_back(0);

             //debuging
             ROS_INFO("approaching to (%lf, %lf)", traj.x.at(traj.index),traj.y.at(traj.index));
             std::stringstream debug_cntrle;
             debug_cntrle<<"approaching to "<<traj.index<< ": ("<< traj.x.at(traj.index)<<", "<< traj.y.at(traj.index)<<")";
             debugger(debug_cntrle.str());
             controller_Status=EXECUTING;
             control(traj_msg);


 }

void controller::control(vector<double> dmsg){


    setpoint.clear();

    for (int i=0;i<4;i++){
        if(!EnableCircle)
        setpoint.push_back(dmsg.at(i));
    else
         setpoint.push_back(0);

    }

    if((controller_Status==IDLE||controller_Status==SUCCESS) && !testingMode ){
        //controller_Status=EXECUTING;
        ROS_INFO_STREAM(dmsg.at(0)<< "\t" <<dmsg.at(1)<< "\t"  <<dmsg.at(2)<< "\t" <<dmsg.at(3));
    }
    else if(testingMode)
     ROS_WARN("Testing motion");
    else
     ROS_WARN("Controller busy");

}

 //controller function
void controller::HoveringMode(){
    Eigen::Vector4d cmd;
    for(int i=0;i<4;i++)
        cmd(i)=0;

    //AVOID OBSTACLES
    switch(obstacleStatus){
        case 2: cmd(0)=UNITSTEP; ROS_INFO("MOVING RIGHT");  break; //   LEFT IS OCCUPIED ROLL RIGHT
        case 3: cmd(0)=-UNITSTEP;ROS_INFO("MOVING LEFT");break; //   RIGHT IS OCCUPIED ROLL LEFT
        case 5: land_pub.publish(std_msgs::Empty());ROS_INFO("LANDING....");break; //   LAND

    }


    ControlCommand c;
    c.roll=cmd(0);   //moving X direction
    c.pitch=cmd(1); //moving Y direction
    c.gaz=cmd(2);    //moving Z direction
    c.yaw=cmd(3); //change Yaw
    cmdPublish(c);
}

void controller::run(const ros::TimerEvent& e){

    if(!state_update && !compute_X_error())
    {HoveringMode();
        return;}
    if(controller_Status==IDLE || controller_Status!=EXECUTING ||testingMode)return;
    Eigen::Vector4d u_cmd= Ar_drone_input();
    ControlCommand c;
    c.roll=u_cmd(0);   //moving X direction
    c.pitch=u_cmd(1); //moving Y direction
    c.gaz=u_cmd(2);    //moving Z direction
    c.yaw=u_cmd(3); //change Yaw
    cmdPublish(c);
    inputApply=true;
    state_update=false;
 }

 void controller::stateUpdate(vector<double> tmsg){
     //x y z Yaw dt

     if(tmsg.size()<14)debugger("controller Error in state update");
//    ROS_INFO_STREAM("state size "<<tmsg.size());
    mutex.lock();
        for(int i=0;i<4;i++)
            X(i) = tmsg.at(i);
    //   Add velocity;

        if(tmsg.size()<8)return;
        for(int j=4;j<8;j++)
            velocity.at(j-4)=tmsg.at(j);
    //   Add acceleration;

        if(tmsg.size()<12)return;
        for(int k=8;k<12;k++)
            acceleration.at(k-8)=tmsg.at(k);

    //update orientation
        ori.roll=tmsg.at(12);
        ori.pitch=tmsg.at(13);
        ori.yaw=tmsg.at(3);
    state_update=true;
    mutex.unlock();


 }


 //compute

 double controller::prediction(int i){
     double phi,psi,theta,alt;
     phi=c3*input.at(0)-c4*ori.roll;
     theta=c3*input.at(1)-c4*ori.roll;
     alt=c5*input.at(2)-c6*ori.pitch;
     psi=c7*input.at(3)-c8*ori.yaw;
     switch (i){
     case 0: return c1*(cos(psi)*sin(phi)*cos(theta)-sin(psi)*sin(theta))-c2*velocity.at(0);
     case 1: return c1*(-sin(psi)*sin(phi)*cos(theta)-cos(psi)*sin(theta))-c2*velocity.at(1);
     case 2: return alt;
     case 3: return psi;
     }

 }


 Eigen::Vector4d controller::Ar_drone_input(){
    //UPDATE CMD BASED ON PD GAIN
    Eigen::Vector4d cmd=kp*X_error+kd*X_dot_error;

    // ROUNDING INPUTS
    float max=cmd.maxCoeff();
    for(int i=0;i<4;i++)
        if(abs(cmd(i))<0.33*abs(max))// TOO SMALL!
            cmd(i)=0;
        else
            cmd(i)=cmd(i)/abs(cmd(i))*UNITSTEP;

    // MAKE YAW CONTROLLER SLAGGISH
    double max_yaw=10*degree;
    if (abs(X_error(3))<max_yaw)
        cmd(3)=0;
    else
         cmd(3)=X_error(3)/abs(X_error(3));

    // IF ROBOT FINDS THE DESTINATION OR OBSTACLE DETECTED
    if( goalConverage()||obstacleStatus)
        for(int i=0;i<4;i++)
            cmd(i)=0;

    //AVOID OBSTACLES
    /*
            c.roll=u_cmd(0);   //moving X direction
            c.pitch=u_cmd(1); //moving Y direction
            c.gaz=u_cmd(2);    //moving Z direction
            c.yaw=u_cmd(3); //change Yaw
     */
    switch(obstacleStatus){
        case 2: cmd(0)=UNITSTEP;break; //   LEFT IS OCCUPIED ROLL RIGHT
        case 3: cmd(0)=-UNITSTEP;break; //   RIGHT IS OCCUPIED ROLL LEFT
        case 5: land_pub.publish(std_msgs::Empty());break; //   LAND

    }


    // RECORD STATE & CMD TO A TXT FILE
    input.clear();
    for(int i=0;i<4;i++)
        input.push_back(cmd(i));
    dataWrite();

    return cmd;

}

 bool controller::compute_X_error(){
     if(setpoint.empty())return false;
     for(int i=0;i<4;i++){
        //update state with input
        // X(i)+=0.5*pow(0.03,2)* prediction(i);
         //compute error
         X_error(i)=setpoint.at(i)-X(i);
     }
     if(!velocity.empty())
          for(int i=0;i<4;i++){
              X_dot_error(i)=-velocity.at(i);
              //X_dot_error(i)-= 0.03*prediction(i);
          }
     return true;


  }

 bool controller::goalConverage(){
    double error=0;
    for (int i=0;i<2;i++)
        error+=pow(setpoint.at(i)-X(i),2);
        //check goal radius
    if(sqrt(error)<STOP){

         ROS_INFO_STREAM("Target "<< sqrt(error) <<" Achieved "<< resetController);
        resetController+=1;
        if(resetController%5==0){
          //  debugger("goal converged");

        }
        if(resetController>30){
            controller_Status==SUCCESS;
            traj.index+=1;
            resetController=0;
            if(traj.index<traj.x.size())
                executingTraj();
           // debugger("goal converged");
        }

        return true;}
        //check unstable vehicle
    else if(abs(velocity.at(0)>MAX_VEL)||abs(velocity.at(1)>MAX_VEL)){
        ROS_INFO_STREAM("stablizing system ");
        return true;}
    else
        return false;
}


 // published command (output)
 void controller::cmdPublish(ControlCommand cmd){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = cmd.yaw;
    cmdT.linear.z = cmd.gaz;
    cmdT.linear.x = cmd.pitch;
    cmdT.linear.y = -cmd.roll;

    mutex.lock();
    vel_pub.publish(cmdT);
    mutex.unlock();

}


//----------------------------------STORE PID GAIN-----------------------------------------------------------------
void controller::wrtieGain(){
    QFile file("/home/redwan/Desktop/pidtune.txt");
    if (file.open(QIODevice::WriteOnly)){
        QStringList list;
        QTextStream outStream(&file);
        for (int i=0;i<8;i++)
            outStream<<gain[i]<<"\n";
         debugger("pidtune new gains are written ");
    }
    else
        debugger("pidtune.txt cannot write ");
    file.close();
}

void controller:: readGain(){
    QFile file("/home/redwan/Desktop/pidtune.txt");
    if (!file.exists())
    {
        gain[0]=100*KpX;gain[2]=100*KpY;gain[4]=100*KpZ;gain[6]=100*KpS;
        gain[1]=100*KdX;gain[3]=100*KdY;gain[5]=100*KdZ;gain[7]=100*KdS;
        wrtieGain();
        sleep(1);
    }

    if (file.open(QIODevice::ReadOnly)){
        QStringList list;
        while (!file.atEnd()) {
              list.append(file.readLine());
          }
        for (int i=0;i<8;i++)
            gain[i]=list[i].toDouble();

        updateGain();


    }
    else
        debugger("pidtune.txt cannot read ");
    file.close();
}

void controller::updateGain(){

    kp<<gain[0]/100.00,0,0,0,0,gain[2]/100.00,0,0,0,0,gain[4]/100.00,0,0,0,0,gain[6]/100.00;
    kd<<gain[1]/100.00,0,0,0,0,gain[3]/100.00,0,0,0,0,gain[5]/100.00,0,0,0,0,gain[7]/100.00;
    debugger("pidtune gain update successfully");
}
//---------------------------------------GUI COMMUNICATION and DATA LOG------------------------------------------------------------
void controller::debugger(std::string ss){
    debug_cntr.data=ss;
    debugger_cntrl.publish(debug_cntr);
    ROS_INFO_STREAM(ss);
}

void controller::dataWrite(){
    QFile file(fileName);
                file.open(QIODevice::Append | QIODevice::Text);

                if(file.isOpen()){
                      QTextStream outStream(&file);
                      //time stamp
                     outStream<< getMS()<<"\t";
                     //robot position
                    for(int i=0;i<4;i++)
                           outStream<<X(i)<<"\t";
                    //controller output
                      for(int i=0;i<4;i++)
                           outStream<<input.at(i)<<"\t";
//                       for(int i=0;i<4;i++)
//                           outStream<<velocity.at(i)<<"\t";
//                       for(int i=0;i<4;i++)
//                           outStream<<acceleration.at(i)<<"\t";

//					   for(int i=0;i<4;i++)
//							outStream<< setpoint.at(i)<<"\t";
                         outStream<<"\n";
                }
                file.close();
}

