#ifndef ROS_THREAD_H
#define ROS_THREAD_H

#include <QThread>
#include <QtCore>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
//#include "moveit_ardrone/posePlan.h"



class ros_thread : public QThread
{
    Q_OBJECT
public:
    explicit ros_thread(QObject *parent = 0);
    void run();
    void sendLand();
    void sendTakeoff();
    void sendToggleState();
    void sendToggleCam();
    void sendFlattrim();
    void posePub(QPointF);
    void goalPub(QPointF);







signals:


    
public slots:


protected:






private:
    QMutex mutex;
    QImage* _image;
    ros::NodeHandle nh_;
    ros::Publisher vel_pub;
    ros::Publisher goal_pub;
    ros::Publisher pose_pub;
    ros::Subscriber navdata_sub;
    ros::Subscriber image_sub;
    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    ros::Publisher toggleState_pub;
    ros::ServiceClient toggleCam_srv;
    std_srvs::Empty toggleCam_srv_srvs;
    ros::ServiceClient flattrim_srv;
    ros::ServiceClient pose_srv;
    std_srvs::Empty flattrim_srv_srvs;
//    moveit_ardrone::posePlan *moveit_srv;



    
};

#endif // ROS_THREAD_H
