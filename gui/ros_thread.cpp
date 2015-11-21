#include "header.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
ros_thread::ros_thread(QObject *parent) :
    QThread(parent)
{

}

void ros_thread::run()
{



    qDebug() << "Starting ROS Publisher Thread";


    vel_pub	   = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("cmd_vel"),1);
    takeoff_pub	   = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/takeoff"),1);
    land_pub	   = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/land"),1);
    toggleState_pub	   = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/reset"),1);
    toggleCam_srv        = nh_.serviceClient<std_srvs::Empty>(nh_.resolveName("ardrone/togglecam"),1);
    flattrim_srv         = nh_.serviceClient<std_srvs::Empty>(nh_.resolveName("ardrone/flattrim"),1);

    goal_pub	   = nh_.advertise<geometry_msgs::Twist >(nh_.resolveName("cmd_vel"),1);
    pose_pub	   = nh_.advertise<geometry_msgs::PoseStamped>("/uav/pose",1);
    qDebug()<<"signal received";
    ros::spin();


}



void ros_thread::goalPub(QPointF msg){

    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0; //yaw
    cmdT.linear.z = 0.0; //alt
    cmdT.linear.x =-msg.x();
    cmdT.linear.y =-msg.y();

     mutex.lock();
        goal_pub.publish(cmdT);
     mutex.unlock();

}

void ros_thread::posePub(QPointF msg){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0; //yaw
    cmdT.linear.z = 0.0; //alt
    cmdT.linear.x =-msg.x();
    cmdT.linear.y =-msg.y();

     mutex.lock();
        goal_pub.publish(cmdT);
     mutex.unlock();

}



void ros_thread::sendLand()
{
    ROS_INFO("sendLand");
    mutex.lock();
    land_pub.publish(std_msgs::Empty());
    mutex.unlock();
}
void ros_thread::sendTakeoff()
{
    ROS_INFO("sendTakeoff");
    mutex.lock();
    takeoff_pub.publish(std_msgs::Empty());
    mutex.unlock();
}
void ros_thread::sendToggleState()
{
    ROS_INFO("sendToggleState");
    mutex.lock();
    toggleState_pub.publish(std_msgs::Empty());
    mutex.unlock();
}
void ros_thread::sendToggleCam()
{
    ROS_INFO("sendToggleCam");
    mutex.lock();
    toggleCam_srv.call(toggleCam_srv_srvs);
    mutex.unlock();
}
void ros_thread::sendFlattrim()
{
    ROS_INFO("sendFlattrim");
    mutex.lock();
    flattrim_srv.call(flattrim_srv_srvs);
    mutex.unlock();
}
