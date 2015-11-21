#ifndef ROS_LAUNCH_H
#define ROS_LAUNCH_H
#include "header.h"

#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

//point cloud header file
//#include "pcl/io/pcd_io.h"
//#include "pcl/point_types.h"
//#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/macros.h>
#include <sensor_msgs/Image.h>
#include <QThread>
#include <QImage>
#include <QDebug>
#include <QMutex>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "octomap_msgs/Octomap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <octomap_msgs/conversions.h>



using namespace std;
class ros_launch : public QThread
{
    Q_OBJECT

public:
    explicit ros_launch(QObject *parent = 0);
    void run();
    void callback_Image(const sensor_msgs::Image::ConstPtr& );
    void debugger_callback(const std_msgs::StringConstPtr);
    void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
    bool Stop;
    std::string image_topic;


signals:
     void ImageQ(const QImage&);
     void singal_sensor_sub(const QImage&);
     void ardrone_battery(double);
     void nav_battery(double);
     void sig_debugger(QString);
     void sig_main_debugger(QString);


    
public slots:
     void slot_ros_launch(const QImage&);
     void slot_nav_battery(double);
     void slot_debugger(QString);

private:
     ros::NodeHandle nh_;
     ros::Subscriber image_sub;
     ros::Subscriber navdata_sub;
     ros::Subscriber debugger_sub;
     ros::Publisher debugger_pub;

     unsigned int navdataCount;
     QMutex mutex;
     QImage _image;






};

#endif // ROS_LAUNCH_H
