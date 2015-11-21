#include "octomap_search.h"


item candidates,path,occupied;

octomap_search::octomap_search()
{

    viz =new display();
    debugger_cntrl=nh_.advertise<std_msgs::String>("jaistquad/debug",1);


    ros::NodeHandle private_nh("~");
    //robot
    private_nh.param<double>("robot_colision_x", rob_col_x, 0.0);
    private_nh.param<double>("robot_colision_y", rob_col_y, 0.0);
    private_nh.param<double>("robot_colision_z", rob_col_z, 0.45);
    //exploration
    private_nh.param<double>("colision_check_per_rob_size", colision_check_per_rob_size, 2.0);
    private_nh.param<double>("explore_z_up_limit", explore_z_up_limit, 0.55);
    private_nh.param<double>("explore_z_down_limit", explore_z_down_limit, 0.25);
    private_nh.param<double>("explore_max_range", explore_max_range, 20.0);
    private_nh.param<double>("explore_min_range", explore_min_range, 0.5);
    private_nh.param<double>("explore_an_shift", explore_an_shift, 20.0);
//    private_nh.param<std::string>("set_goal_topic", set_goal_topic, string("/set_exploration_goal"));
    private_nh.param<double>("max_x_world_size", max_x_world_size, 40.0);
    private_nh.param<double>("max_y_world_size", max_y_world_size, 40.0);
    private_nh.param<int>("explore_rate", explore_rate, 2.0);
    //private_nh.param<std::string>("map_2d_topic", map_2d_topic, string("/projected_map"));
    private_nh.param<bool>("show_markers", show_markers, true);
//    private_nh.param<std::string>("exploration_strategy", exploration_strategy, string("closest_goal"));
    private_nh.param<double>("goal_offset", goal_offset, 0.10);

    //trajectory publisher
    traj= nh_.advertise<geometry_msgs::PoseArray>("traj", 5);



}


void octomap_search::updateRobot(vector<float> pos){
    rob_col_x=pos.at(0);
    rob_col_y=pos.at(1);
//    rob_col_z=pos.at(2);

}

void octomap_search::check_position_colision(const geometry_msgs::PoseStamped pose)
{
//defina area search
double octree_res = octree->getResolution();
int zd0 =z_limiter= (pose.pose.position.z - rob_col_z/2)*(1/octree_res);
int zd1 = (pose.pose.position.z + rob_col_z/2)*(1/octree_res);
pose_occupied = false;
double x, y, trackingNumber=0;



//candidates.clear();
//shortest path
double dist=sqrt(pow(rob_col_x-goal_col_x,2)+pow(rob_col_y-goal_col_y,2));
dist=round(dist+1);
double zero=0;
double theta=360*deg+atan((goal_col_y)/(goal_col_x));
double x_left=0,x_right=180*deg;

for (float an=0; an<=180*deg; an+=10*deg){

//    if(an>=x_left && an<=x_left)

    for (float j=0;j<=dist;j+=0.5){
        double x=rob_col_x+j*cos(an);
        double y=rob_col_y+j*sin(an);



        oc_node = octree->search((double)x, (double)y, (double)zd0*octree_res);
           if(oc_node)
            {
                if(oc_node->getValue() > 0)
                {
                    if(x<rob_col_x)
                        x_left=an;
                    else
                        x_right=an;

                }
                else
                {
                    std::vector<float> points;
                    points.push_back(x);points.push_back(y);
                    points.push_back(1);points.push_back(2);
                    candidates.push_back(points);
                }
            }

    }


}





return;


}

bool octomap_search::call_octomap()
{
    ROS_INFO("Exploration: octomap callback");
    std::string servname = "/octomap_full";
    octomap_msgs::GetOctomap::Request req;
    octomap_msgs::GetOctomap::Response resp;

    if(!ros::service::call(servname, req, resp)){
        debugger("requested octomap failed");
        return false;
    }
//    while(nh_.ok() && !ros::service::call(servname, req, resp))
//    {
//    ROS_WARN("Request to %s failed; trying again...", nh_.resolveName(servname).c_str());
//    usleep(1000000);
//    }
    if (nh_.ok())
    { // skip when CTRL-C
    AbstractOcTree* tree=octomap_msgs::msgToMap(resp.map);
    octree = dynamic_cast<octomap::OcTree*>(tree);
    co_octomap++;
    if(co_octomap == 1)
    {
    octree_res = octree->getResolution();
//    ROS_INFO("Tree resolution: %f Octree size: %i", octree_res, octree->size());
    double x,y,z;
    octree->getMetricSize(x, y, z);
//    ROS_INFO("Tree metric size: x: %f y: %f z: %f", x,y,z);
    }
    }
    return true;
}

void octomap_search::find_status(){
    candidates.clear();
    occupied.clear();
    if(!call_octomap())return;
    ROS_INFO("Exploration: Updated Octomap Received!");
    //set borders
    int zd0 = explore_z_down_limit*(1/octree_res);
    int zd1 = explore_z_up_limit*(1/octree_res);
    int rd1 = round(explore_max_range*(1/octree_res));
    int rd0 = round(explore_min_range*(1/octree_res));
    //explore init
    double x=0,y=0;
    double tmp_ex_goals[3];
    int co_unknown = 0;




    geometry_msgs::PoseStamped col_pose;
    bool skip_pose = false;
    //find the frontears
    ROS_INFO_STREAM("find the frontears "<<zd0<<"\t"<<zd1<<"\t"<<rd1<<"\t"<<rd0<<"\t"<<octree_res);
    for(int z=zd0; z < zd1; z++)
    {
    z = z + (rob_col_z/2)*(1/octree_res);
    for (int an=0; an<361; an++)
    {
        an = an + (int)explore_an_shift; //search angle
        skip_pose = false;
        for(int r=rd0; r < rd1; r++)
        {
        //increase distance
        r = r + rob_col_x/colision_check_per_rob_size*(1/octree_res);
        x = r*octree_res * cos(an*0.0174532925)+curr_pose_x;
        y = r*octree_res * sin(an*0.0174532925)+curr_pose_y;
//        ROS_INFO("search in X: %f Y: %f Z: %f", (double)x, (double)y, (double)z*octree_res);
        oc_node = octree->search((double)x, (double)y, (double)z*octree_res);
            if(oc_node)
            {
                if(oc_node->getValue() > 0)
                {
                    vector<float>pa;

                    pa.push_back(x);
                    pa.push_back(y);
                    pa.push_back(3);
                    occupied.push_back(pa);
                //oc_node->setValue(-2.0);
                //oc_node->setColor(0, 255, 0);
//                ROS_INFO("Node is OCCUPIED!");
                }


    else
    {
//    ROS_INFO("Free in X: %f Y: %f Z: %f", (double)x, (double)y, (double)z*octree_res);
    col_pose.pose.position.x = x;
    col_pose.pose.position.y = y;
    col_pose.pose.position.z = z*octree_res;
    check_position_colision(col_pose); //check if there are colisions
    if(pose_occupied == false)
    {
    tmp_ex_goals[0] = x;
    tmp_ex_goals[1] = y;
    tmp_ex_goals[2] = z*octree_res;
//    ROS_INFO("Potential goal: %f %f %f", x, y, z*octree_res);
    }else
    {
    skip_pose = true;
    }
    }
    co_unknown = 0;
    }else
    {
    //this cell is unknown, store old one
    co_unknown++;
    if(co_unknown > 60)
    {
//    ROS_INFO("Found unknown area (%lf %lf %lf) ",x,y,z*octree_res);
    co_unknown = 0;
    break;
    }
    }
    }
}
    }

    ROS_INFO_STREAM("candidates size "<<candidates.size());


 }

void octomap_search::foundPath(vector<float>goal){

    ROS_INFO("robot (%f %f ) goal (%f %f)", rob_col_x, rob_col_y, goal.at(0),goal.at(1));
    goal_col_x=goal.at(0); goal_col_y=goal.at(1);
     std::stringstream debug_cntrle;
     debug_cntrle<<"From ("<<rob_col_x<<", "<<rob_col_y<<") to ("<<goal.at(0)<<", "<<goal.at(1)<<")";
    debugger(debug_cntrle.str());
    float scale=2.00;
    dstar = new Dstar();
    dstar->init(0,0, 50000,50000);
    dstar->updateStart(scale*rob_col_x, scale*rob_col_y);
    dstar->updateGoal(scale*goal.at(0),scale*goal.at(1));


    ROS_INFO_STREAM("occupied size "<<occupied.size());

    ROS_INFO_STREAM("frontier size "<<candidates.size());


    int canCount=0;
    path.clear();

    foreach(vector<float>p,candidates){
       dstar->updateCell(scale*p.at(0),scale*p.at(1),1/exp(p.at(3)) );

       canCount++;
        if(canCount>3000-1){
            ROS_WARN("SOLUTION does not converage");
            break;}
    }
    foreach(vector<float>p,occupied)
             dstar->updateCell(scale*p.at(0),scale*p.at(1),-1);

    dstar->replan();
    if(dstar->getPath().empty())
            ROS_WARN("No path found");
    else{
    foreach(state p, dstar->getPath()){
        vector<float>pa;
        float x(p.x/scale),y(p.y/scale);
        pa.push_back(x);
        pa.push_back(y);
        pa.push_back(3);
        path.push_back(pa);
        candidates.push_back(pa);
    }



    ROS_INFO_STREAM("path size "<<path.size());
    if(!candidates.empty()){

        viz->obstacleMap( candidates);
        publishTraj();
    }
    else
        debugger("NO path found;");

//    viz->obstacleMap( path);

    }


}

void octomap_search::publishTraj(){
    geometry_msgs::PoseArray trajArray;
     std::stringstream debug_cntrle;
    int i=0;
    foreach (std::vector<float>p,path){
         i++;
         if(i>1){
             geometry_msgs::Pose pose;
             pose.position.x=p.at(0);
             pose.position.y=p.at(1);
             trajArray.poses.push_back(pose);
             debug_cntrle<<"Target " << i-1<< ": ("<<p.at(0)<<", "<<p.at(1)<<")\n";
         }

    }
    traj.publish(trajArray);
    debugger(debug_cntrle.str());
}

void octomap_search::debugger(std::string ss){

    debug_cntr.data=ss;
    debugger_cntrl.publish(debug_cntr);
    ROS_INFO_STREAM(ss);


}

void octomap_search::goalPublisher(float x, float y){
     geometry_msgs::PoseArray trajArray;
      geometry_msgs::Pose pose;
      pose.position.x=x;
      pose.position.y=y;
      trajArray.poses.push_back(pose);
      traj.publish(trajArray);
      debugger("new setpoint is published");
 }


bool octomap_search::octree_search(double x, double y){

    oc_node = octree->search((double)x, (double)y, (double)z_limiter*octree_res);
       if(oc_node)
        {
            if(oc_node->getValue() > 0)
            {
                return true;
            }

        }


       return false;
}



/*
 * obstacle avoidance function
 */
void octomap_search::obstacle_avoid(double*A,double rx,double ry){

    A[0]=A[1]=0;



    for (float an=0; an<=360*deg; an+=10*deg){

            double x=rx+cos(an);
            double y=ry+sin(an);
           if(octree_search(x,y)){

                   if(   0<an && an<=45  )      A[0]+=1;
                   else if(  45<an && an<=135 )    A[1]-=1;
                   else if( 135<an && an<=225 )   A[0]-=1;
                   else if( 225<an && an<=315 )   A[1]+=1;
                   else if( 315<an && an<=360 )   A[0]+=1;

           }
    }




    ROS_INFO_STREAM("obs "<<A[0]<<"\t"<<A[1]);
    A[0]/=abs(A[0]);
    A[1]/=abs(A[1]);

}


