#include "display.h"

display::display()
{
    robot_pub= nh_.advertise<visualization_msgs::Marker>("HexTree/sample", 1);
    robot.header.frame_id=dpoptimal.header.frame_id=plan.header.frame_id =track_list.header.frame_id = "Quad";
    robot.header.stamp=dpoptimal.header.stamp=plan.header.stamp =track_list.header.stamp = ros::Time::now();
    robot.ns=plan.ns ="sample";
    track_list.ns = "path";
    dpoptimal.ns="optimalPath";

    robot.id = 0;dpoptimal.id=1;
    robot.type  = visualization_msgs::Marker::ARROW;
    robot.action =dpoptimal.action=track_list.action = visualization_msgs::Marker::ADD;

    robot.scale.x  = 0.5;
    robot.scale.y  = 0.5;
    robot.scale.z  = 0.1;


    dpoptimal.type= visualization_msgs::Marker::LINE_STRIP;
    dpoptimal.scale.x  = 0.25;
    dpoptimal.scale.y  = 0.25;
    dpoptimal.scale.z  = 0.1;
    // Robot is red
    robot.color.r = 1.0f; dpoptimal.color.b =dpoptimal.color.r= 1.0f;
    robot.color.a =dpoptimal.color.a= 1.0;
    robot.lifetime = ros::Duration();

    //configure samplePlan
    plan.pose.orientation.w = 1.0;
    plan.id =hexID=0;
    plan.type = visualization_msgs::Marker::POINTS;
    plan.lifetime=ros::Duration();

    plan.scale.x = 0.15;
    plan.scale.y = 0.15;
    plan.scale.z = 0.15;

    // samplePlan is red
    plan.color.r = 1.0;
    plan.color.a = 1.0;


    //configure track list
    track_list.pose.orientation.w = 1.0;
    track_list.id = 0;
    track_list.type = visualization_msgs::Marker::LINE_STRIP;

    track_list.scale.x = 0.15;
    track_list.scale.y = 0.15;
    track_list.scale.z = 0.15;
    // track list is blue
    track_list.color.b = 1.0;
    track_list.color.a = 1.0;


}

//PUBLISHER
void display::hexSample(float SamplePoits[7][2], float localPathIndex[6]){

    SamplePoits[6][0]=SamplePoits[0][0];
    SamplePoits[6][1]=SamplePoits[0][1];

//    ROS_INFO_STREAM("planID "<< plan.id<< "hexID "<<hexID);
   for (int i=0;i<7;i++)
    {
      geometry_msgs::Point p;
      p.x = SamplePoits[i][0];
      p.y = SamplePoits[i][1];
      p.z = 1;
      plan.points.push_back(p);
    }
     //publish hexagonal sample location
     robot_pub.publish(plan);
     // publish sampling path
     for (int i=0;i<6;i++)
      {
         if(localPathIndex[i]==-1)continue;
         int index=localPathIndex[i];
//         printf("%d\t",index);
        geometry_msgs::Point p;
        p.x = SamplePoits[index ][0];
        p.y = SamplePoits[index][1];
        p.z = 1;
        track_list.points.push_back(p);
      }
//     printf("\n");

     robot_pub.publish(track_list);
     hexID +=1;
     plan.id=track_list.id=hexID;

}

 void display::optimalPath(float *x,float*y,int count){

    dpoptimal.points.clear();
     for (int i=0;i<count;i++)
      {
        geometry_msgs::Point p;
        p.x = x[i];
        p.y = y[i];
        p.z = 1;
        dpoptimal.points.push_back(p);
      }

     robot_pub.publish(dpoptimal);
     sleep(1);
 }
