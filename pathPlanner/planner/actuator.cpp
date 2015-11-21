#include "actuator.h"
#include "Dstar.h"
#include "tum_ardrone/filter_state.h"

item updatedmap;

actuator::actuator()
{
    mapExist=false;
    CLOUDSCALE=alt_off=1;
    dronepose_pub	   = nh_.advertise<tum_ardrone::filter_state>(nh_.resolveName("ardrone/predictedPose"),1);
    pcl_pub2=nh_.advertise<sensor_msgs::PointCloud2>("Quad/map",10);
    map_modify=nh_.advertise<sensor_msgs::PointCloud2>("Quad/modify_map",10);

}


//RELATED TO PATH PLANNING
void actuator::run(TODPOSITION robo ,TODPOSITION goal){

    robo=TODPOSITION{robo.x*PLANNINGSCALE,robo.y*PLANNINGSCALE};
    goal=TODPOSITION{goal.x*PLANNINGSCALE,goal.y*PLANNINGSCALE};

    cout<< "robot{"<<robo.x<<" "<<robo.y<<"}\t goal{"<< goal.x<<" "<<goal.y<<"}"<<endl;
    if(!mapExist)
        ROS_ERROR("no plan found");


    //    dstar = new Dstar();
//    viz=new display();
//    dstar->init(robo.x,robo.y, goal.x,goal.y);
//    if(mapExist)
//        searchSpace();


////        update goal position if required
////        dstar->updateGoal(mG.x(),mG.y());
//        dstar->replan();
//         foreach(state p, dstar->getPath())
//             mypath.push_back( state{p.x/PLANNINGSCALE,p.y/PLANNINGSCALE});


//        //may be print is required to understand
////        printPath();
//        return mypath;
}

 void actuator::map(){
    mapExist=true;
}

 void actuator::searchSpace(){

}

 void actuator::printPath(){
    foreach (state path,mypath){
        cout<< "path{"<< path.x<<" "<<path.y<<"}"<<endl;
    }
}

// RELATED TO STATE PUBLISHING

 void actuator::statePublish(vector<double> state){
    tum_ardrone::filter_state s;

    s.header.stamp = ros::Time().now();
//    POSITION
    s.x=state.at(0);
    s.y=state.at(1);
    s.z=state.at(2);
//    VELOCITY
    s.dx=state.at(3);
    s.dy=state.at(4);
    s.dz=state.at(5);

//    ORIENTATION
    s.roll=state.at(6);
    s.pitch=state.at(7);
    s.yaw=state.at(8);
    s.dyaw=state.at(9);

//    UTILITIES
    s.batteryPercent=state.at(10);
    s.droneState=state.at(11);
    s.ptamState=5;//PTAM TOOK KF

    dronepose_pub.publish(s);

 }

 void actuator::updateCloudScale(double scale, double offset){
     CLOUDSCALE=scale;
     alt_off=offset;
 }

 void actuator::pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
 {
     mutex.lock();
//     cout<<"publishing cloud"<<endl;
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(*input,pcl_pc2);
     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);


     for(size_t i=0;i<temp_cloud->points.size();i++)
      {
 //       convert to eigen vector
        Eigen::Vector3d p(temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z);
        Eigen::Transform<double,3,Affine>  R;
        R=AngleAxisd(M_PI/2, Vector3d::UnitX())*AngleAxisd(M_PI, Vector3d::UnitY());
        p=CLOUDSCALE*R.rotation()*p;
        temp_cloud->points[i].x=-p(0);
        temp_cloud->points[i].y=p(1);
        temp_cloud->points[i].z=p(2)+alt_off;
     }

     sensor_msgs::PointCloud2 pcl_msg;
     pcl::toROSMsg(*temp_cloud,pcl_msg);
     const char* MAP_FRAME_ID = "Quad";
     pcl_msg.header.frame_id = MAP_FRAME_ID;
     pcl_msg.header.stamp  = ros::Time::now();


     pcl_pub2.publish(pcl_msg);
     mutex.unlock();

 }

 void actuator::writemap(const nav_msgs::OccupancyGridPtr msg){

//     nav_msgs::MapMetaData info(msg->info);
//     geometry_msgs::Pose origin(info.origin);


//     mutex.lock();
//     map2D.height=info.height;
//     map2D.width=info.width;
//     map2D.originX=origin.position.x;
//     map2D.originY=origin.position.y;
//     for(int width=0;width<info.width;width++)
//         for(int height=0;height<info.height;height++)
//                 map2D.grid[width][height]=msg->data[width+width*height];

//      mutex.unlock();
//      obstacleMapPublish();



//     ROS_INFO("Exploration: Map callback");
//    //get image
//     double width = msg->info.width;
//     double height = msg->info.height;
//     double resolution_ = msg->info.resolution;
//     //ROS_INFO("Resolution: %f", resolution_);
//     cv::Mat free_space( width, height, CV_8UC1);
//     // build image from occupancy grid
//     for(uint32_t row = 0; row < height; row++)
//     {
//     for(uint32_t col = 0; col < width; col++)
//     {
//     int8_t calcIndex =(height-1-row)*width+col;
//     int8_t map_value = msg->data[calcIndex];
//     if( map_value == 0 ) // free space is set to white
//     {
//     ((uchar*)(free_space.data + free_space.step*col))[row] = 255;
////         free_space.at<uchar>(col,row)=255;


//     }
//     else // occupated space or unknown is set to black
//     {
//         ((uchar*)(free_space.data + free_space.step*col))[row] = 0;
////     ((uchar*)(free_space.data + free_space.step*row))[col] = 0;
////          free_space.at<uchar>(col,row)=0;
//     }
//     }
//     }
//     cv::imshow("fres space map", free_space);
//     cv::waitKey(3);




 }

 void actuator::obstacleMapPublish(){
     item obsgrid;

     for(int width=0;width<map2D.width;width++)
         for(int height=0;height<map2D.height;height++){
             //if obstacle exist
             if(map2D.grid[width][height]==100){
                 //calculate its cordinate
                 std::vector<float> obs;
                 obs.push_back(height+map2D.originY*10);
                 obs.push_back(width+map2D.originX*10);
                 obsgrid.push_back(obs);
             }

         }
//     viz->obstacleMap(obsgrid);

     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
     temp_cloud->resize(obsgrid.size());
     int count=0;
         foreach (std::vector< float > obs, obsgrid){
             temp_cloud->points[count].y=0.05*obs.at(0);
             temp_cloud->points[count].x=0.05*obs.at(1);
             temp_cloud->points[count].z=0;
             count+=1;

         }
        obsgrid.clear();
     sensor_msgs::PointCloud2 pcl_msg;
     pcl::toROSMsg(*temp_cloud,pcl_msg);
     const char* MAP_FRAME_ID = "Quad";
     pcl_msg.header.frame_id = MAP_FRAME_ID;
     pcl_msg.header.stamp  = ros::Time::now();
     map_modify.publish(pcl_msg);

 }
