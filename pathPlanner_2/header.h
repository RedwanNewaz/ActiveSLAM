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
#include "sensor_msgs/Imu.h"

#include <QMutex>


// state estimation
#include "stateEstimator/stateestimation.h"
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>


//octomap
#include <octomap_msgs/GetOctomap.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/GetOctomapResponse.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/BoundingBoxQueryRequest.h>
#include <octomap_msgs/BoundingBoxQueryResponse.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap/OcTreeStamped.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/CountingOcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTreeNode.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Utils.h>
#include "planner/octomap_search.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


using namespace cv;
using namespace std;


