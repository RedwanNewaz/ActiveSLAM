#ifndef OCTOMAP_SEARCH_H
#define OCTOMAP_SEARCH_H
#include "../header.h"
#include "Dstar.h"
#include "octomap/AbstractOcTree.h"
#include"octomap_msgs/conversions.h"
#include <octomap_msgs/GetOctomap.h>
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/GetOctomapResponse.h"
#include "octomap_msgs/GetOctomapRequest.h"
#include "octomap_msgs/OctomapWithPose.h"
#include "geometry_msgs/PoseArray.h"

#define deg 0.0174532925


using namespace octomap;
class display;
class octomap_search
{
public:
    octomap_search();
    bool call_octomap();
    void evaluateNode();
    void find_status();
    void updateRobot(vector<float>);
    void check_position_colision(const geometry_msgs::PoseStamped pose);
    void getOctomap( octomap_msgs::Octomap::ConstPtr msg );
    void foundPath(vector<float>goal);
    void debugger(std::string ss);
    void goalPublisher(float x, float y);
    Dstar *dstar;
    double z_limiter;
    void obstacle_avoid(double*,double,double);
    bool octree_search(double x, double y);

private:
    //exploration

    bool start_exploration;
    double explore_z_up_limit;
    double explore_z_down_limit;
    double explore_max_range;
    double explore_min_range;
    double explore_an_shift;
    double global_x, global_y, global_z;
    double rob_col_x, rob_col_y, rob_col_z;
    double goal_col_x, goal_col_y, goal_col_z;
    double colision_check_per_rob_size;
    double yaw, pitch, roll;
    double curr_pose_x, curr_pose_y, curr_pose_z;
    bool show_markers;
    bool pose_occupied;
    double max_x_world_size;
    double max_y_world_size;
    int explore_rate;
    int explore_node_co;
    double goal_offset;
    double xmax,ymax,zmax;
    double xmin,ymin,zmin;
    double exploration_poses_history[100][10][3];
    double way_choosed[100][1];
    //octomap
    octomap::OcTree* octree;
    OcTreeNode* oc_node;
    double octree_res;
    int co_octomap;
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub;
    ros::Publisher traj;
    display *viz;


    //debugger
    ros::Publisher debugger_cntrl;

    std_msgs::String debug_cntr;

protected:
    void publishTraj();

};

#endif // OCTOMAP_SEARCH_H
