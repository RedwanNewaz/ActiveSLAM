#include "mav.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "planner/actuator.h"
#include "visualizer/display.h"


#include "ardrone_autonomy/Navdata.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

//point cloud header file
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/macros.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "octomap_msgs/Octomap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <octomap_msgs/conversions.h>
#include "sensor_msgs/Imu.h"

#include <QMutex>
#include <QThread>
#include <QImage>
#include <QDebug>


// state estimation
#include "stateEstimator/stateestimation.h"
#include "stateEstimator/scale.h"
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "boost/thread.hpp"
#include <boost/math/constants/constants.hpp>


#include "visualization_msgs/Marker.h"


