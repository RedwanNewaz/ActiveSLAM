#include "mav.h"
#include "ros/ros.h"



mav::mav()
{
    visualize=new display();
    plan =new actuator();
    timer = nh_.createTimer(ros::Duration(0.5), &mav::timerCallback,this);
    map_search=new octomap_search();
    planner_talk=nh_.advertise<std_msgs::String>("jaistquad/motionplan",1);
    service = nh_.advertiseService("motionplan",&mav::talk,this);





}

mav::~mav()
{

}



void mav::run(){
        ROS_INFO("SENSOR SUBSCRIBTION ENABLE");
        sleep(1);
        map_search->debugger("path planner is intialized");

      //ORB_SLAM DEPENDECIES
        orbTraker=nh_.subscribe("ORB_SLAM/Debug",10,&mav::takerCallback, this);
        cloud_sub=nh_.subscribe("/ORB_SLAM/PointCloud_raw",10,&mav::pointcloudCallback, this);

        //OCTOMAP DEPENDENCIES
        projectedmap_sub=nh_.subscribe("/projected_map",10,&mav::projectedmapCallback, this);

        //RVIZ DEPENDECIES
        rviz_goal=nh_.subscribe("move_base_simple/goal",10,&mav::goalCallback, this);
        rviz_pose=nh_.subscribe("initialpose",10,&mav::poseCallback, this);



}

 void mav::timerCallback(const ros::TimerEvent& e){
     mutex.lock();

     mutex.unlock();

 }

 //user input
 bool mav::talk(active_slam::plannertalk::Request  &req,
          active_slam::plannertalk::Response &res)
 {
    if(req.option==1){
        map_search->debugger("p2p planner motion is selected");
        p2pNav=true;}
    else if(req.option==2){
        p2pNav=false;
        map_search->debugger("traj folower motion is selected");
    }
    res.result=true;
     return true;
 }




 //ORB_SLAM DEPENDECIES
void mav::pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
//        mapPublisher
    mutex.lock();
        plan->pointcloudCallback(input);
    mutex.unlock();

}

void mav::takerCallback(const std_msgs::String &msg){

}

void mav::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr  msg){

    vector<float>robot;
    robot.push_back(msg->pose.pose.position.x);
    robot.push_back(msg->pose.pose.position.y);
    map_search->updateRobot(robot);
   ROS_INFO("pose (%f, %f) ",msg->pose.pose.position.x,msg->pose.pose.position.y);
}


void mav::goalCallback(const geometry_msgs::PoseStampedConstPtr msg){

    if(p2pNav){
        map_search->goalPublisher(msg->pose.position.x,msg->pose.position.y);
//         map_search->debugger("new goal is arrived");
    }
    else{
        vector<float>robot;
        robot.push_back(msg->pose.position.x);
        robot.push_back(msg->pose.position.y);
        map_search->find_status();
        map_search->foundPath(robot);
    }

//    QPointF goal_rviz(msg->pose.position.x,msg->pose.position.y);
//    mutex.lock();


//    double 2Dmap[info.width ][info.height];

//    msg->data[i+ info.width * int(W_t_G.y());

//    mutex.unlock();








}



    //OCTOMAP DEPENDENCIES
void mav::projectedmapCallback(const nav_msgs::OccupancyGridPtr msg){
    plan->writemap(msg);

}
