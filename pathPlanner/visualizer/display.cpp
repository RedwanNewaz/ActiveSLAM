#include "display.h"

display::display()
{
    robot_pub = nh_.advertise<visualization_msgs::Marker>("Quad/state/map", 1000);
    pcl_pub_plan=nh_.advertise<sensor_msgs::PointCloud2>("Quad/map/candidates",10);
    pcl_pub2=nh_.advertise<sensor_msgs::PointCloud2>("Quad/map",10);


    robot.header.frame_id=plan.header.frame_id =track_list.header.frame_id = "Quad";
    robot.header.stamp=plan.header.stamp =track_list.header.stamp = ros::Time::now();
    robot.ns=plan.ns =track_list.ns = "stateEstimation";

    robot.id = 0;
    robot.type = visualization_msgs::Marker::ARROW;
    robot.action =track_list.action = visualization_msgs::Marker::ADD;

    robot.scale.x = 0.1;
    robot.scale.y = 0.1;
    robot.scale.z = 0.1;
    // Robot is red
    robot.color.r = 1.0f;
    robot.color.a = 1.0;
    robot.lifetime = ros::Duration();

    //configure pathPlan
    plan.pose.orientation.w = 1.0;
    plan.id = 2;
    plan.type = visualization_msgs::Marker::LINE_STRIP;

    plan.scale.x = 0.05;
    plan.scale.y = 0.05;
    plan.scale.z = 0.05;


    //  path is pink
    plan.color.b = 1.0;
    plan.color.r = 1.0;
    plan.color.a = 1.0;

    //configure track list
    track_list.pose.orientation.w = 1.0;
    track_list.id = 50;
    track_list.type = visualization_msgs::Marker::POINTS;

    track_list.scale.x = 0.05;
    track_list.scale.y = 0.05;
    track_list.scale.z = 0.05;
    // track list is red
    track_list.color.r = 1.0;
    track_list.color.a = 1.0;


}

//PUBLISHER
void display::obstacleMap( item collision, bool search){
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    temp_cloud->resize(collision.size());
    sensor_msgs::PointCloud2 pcl_msg;
    int count=0;
    float x,y,z;
    foreach (std::vector<float>p,collision){
         x=temp_cloud->points[count].x=p.at(0);
         y=temp_cloud->points[count].y=p.at(1);
         if(search)
         z=temp_cloud->points[count].z= 0.7;//p.at(2);
         else
             z=temp_cloud->points[count].z= p.at(2);
//         ROS_INFO("map element %d : %f  %f  %f ",count,x,y,z);
         count++;
    }

    pcl::toROSMsg(*temp_cloud,pcl_msg);
    const char* MAP_FRAME_ID = "Quad";
    pcl_msg.header.frame_id = MAP_FRAME_ID;
    pcl_msg.header.stamp  = ros::Time::now();
    pcl_pub_plan.publish(pcl_msg);


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

//    if(!plan.points.empty())
//        robot_pub.publish(plan);



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


}

void display::pathViz(std::vector<double> x,std::vector<double> y){

    plan.points.clear();
    for (int i=0;i<x.size();i++){

        geometry_msgs::Point p;
        p.x = x.at(i);
        p.y = y.at(i);
        p.z = 1.2;

        plan.points.push_back(p);
    }
    robot_pub.publish(plan);
}
