#include "display.h"

display::display()
{
    robot_pub =obstacle_pub= nh_.advertise<visualization_msgs::Marker>("Quad/state/map", 1);
//    obstacle_pub = nh_.advertise<visualization_msgs::Marker>("Quad/state/obs", 1);
    pcl_pub2=nh_.advertise<sensor_msgs::PointCloud2>("Quad/map",10);
    debugger_pub=nh_.advertise<std_msgs::String>("jaistquad/debug",1);

    robot.header.frame_id=obstacle.header.frame_id=plan.header.frame_id =track_list.header.frame_id = "Quad";
    robot.header.stamp=obstacle.header.stamp=plan.header.stamp =track_list.header.stamp = ros::Time::now();
    robot.ns=obstacle.ns=plan.ns =track_list.ns = "stateEstimation";

    robot.id = 0;obstacle.id=1;
    robot.type =obstacle.type = visualization_msgs::Marker::ARROW;
    robot.action =obstacle.action=track_list.action = visualization_msgs::Marker::ADD;

    robot.scale.x  = 0.5;
    robot.scale.y  = 0.5;
    robot.scale.z  = 0.1;

    obstacle.scale.x  = 0.25;
    obstacle.scale.y  = 0.25;
    obstacle.scale.z  = 0.1;
    // Robot is red
    robot.color.r = 1.0f; obstacle.color.b = 1.0f;
    robot.color.a =obstacle.color.a= 1.0;
    robot.lifetime = ros::Duration();

    //configure pathPlan
    plan.pose.orientation.w = 1.0;
    plan.id = 2;
    plan.type = visualization_msgs::Marker::LINE_STRIP;

    plan.scale.x = 0.05;
    plan.scale.y = 0.05;
    plan.scale.z = 0.05;

    // track list is blue
    plan.color.r = 1.0;
    plan.color.a = 1.0;


    //configure track list
    track_list.pose.orientation.w = 1.0;
    track_list.id = 2;
    track_list.type = visualization_msgs::Marker::POINTS;

    track_list.scale.x = 0.05;
    track_list.scale.y = 0.05;
    track_list.scale.z = 0.05;
    // track list is blue
    track_list.color.b = 1.0;
    track_list.color.a = 1.0;


}

//PUBLISHER
void display::ukf_transformer( std::vector<double>robot_pose){
    geometry_msgs::Pose pose;


    debug_pose.str("");
    debug_pose<<"robot\t"<<robot_pose.at(0)<<"\t"<<robot_pose.at(1)<<"\t"<<robot_pose.at(2);
    debug_pose<<"\t"<<robot_pose.at(3)<<"\t"<<robot_pose.at(4)<<"\t"<<robot_pose.at(5);

    robo_x=pose.position.x=robot_pose.at(0)*VISFACTOR;
    robo_y=pose.position.y=robot_pose.at(1)*VISFACTOR;
    robo_z=pose.position.z=robot_pose.at(2)*VISFACTOR;


//    ORIENTATION
    double  ro=robot_pose.at(3),
            po=robot_pose.at(4),
            yo=robot_pose.at(5);
//    cout<< setprecision(3)<<ro<<"\t"<<po<<"\t"<<yo;
    Quaterniond q=euler2Quaternion(ro,po,yo+M_PI/2);
    pose.orientation.x=q.x();
    pose.orientation.y=q.y();
    pose.orientation.z=q.z();
    pose.orientation.w=q.w();

    robot.pose=pose;

//    Track Position
    positionTracker.x.push_back(robot_pose.at(0)*VISFACTOR);
    positionTracker.y.push_back(robot_pose.at(1)*VISFACTOR);
    positionTracker.z.push_back(robot_pose.at(2)*VISFACTOR);


    for (uint32_t i = 0; i < positionTracker.x.size(); ++i)
    {
      geometry_msgs::Point p;
      p.x = positionTracker.x.at(i);
      p.y = positionTracker.y.at(i);
      p.z = positionTracker.z.at(i);
      track_list.points.push_back(p);
    }



    robot_pub.publish(robot);
    robot_pub.publish(track_list);


    debug_posi.data=debug_pose.str();
    debugger_pub.publish(debug_posi);


}

void display::mapPublisher(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(*temp_cloud,pcl_msg);
    const char* MAP_FRAME_ID = "Quad";
    pcl_msg.header.frame_id = MAP_FRAME_ID;
    pcl_msg.header.stamp  = ros::Time::now();
    pcl_pub2.publish(input);



}

void display::pathShow(list<state> mypath){

   foreach (state path,mypath)
    {
      geometry_msgs::Point p;
      p.x = path.x;
      p.y = path.y;
      p.z = 1;
      plan.points.push_back(p);
    }
     robot_pub.publish(plan);

}

void display::obstacle_boundary(int* A){
     geometry_msgs::Pose pose;
     pose.position.z=robo_z;


     orientation direction;
     for(int i=0;i<=1;i++){

         if(A[i]==0)continue;

         switch (i) {
             case 0:
             if(A[i]>0){
                direction=RIGHT;
                obstacle.id=1;
             }
             else{
                direction=LEFT;
                obstacle.id=2;
             }
                break;
         case 1:
             if(A[i]>0){
                direction=FORWARD;
                obstacle.id=3;
             }
             else{
                direction=BACKWARD;
                obstacle.id=4;
             }
            break;
     }




     switch (direction){
     case RIGHT:
         pose.position.x=robo_x+1;
         pose.position.y=robo_y;
         pose.orientation.x=pose.orientation.y=0;
         pose.orientation.z=1;
         pose.orientation.w=0;
         break;
     case LEFT:
         pose.position.x=robo_x-1;
         pose.position.y=robo_y;
         pose.orientation.x=pose.orientation.y=0;
         pose.orientation.z=0.0;
         pose.orientation.w=0.707;
         break;
     case FORWARD:
         pose.position.x=robo_x;
         pose.position.y=robo_y+1;
         pose.orientation.x=pose.orientation.y=0;
         pose.orientation.z=-0.707;
         pose.orientation.w=0.707;
         break;
     case BACKWARD:
         pose.position.x=robo_x;
         pose.position.y=robo_y-1;

         pose.orientation.x=pose.orientation.y=0;
         pose.orientation.z=0.707;
         pose.orientation.w=0.707;
         break;
     }


     obstacle.pose=pose;
     obstacle_pub.publish(obstacle);

     }
     }
