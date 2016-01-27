#include "octomap_search.h"


item candidates,path,occupied;

octomap_search::octomap_search()
{

    viz =new display();
    debugger_cntrl=nh_.advertise<std_msgs::String>("jaistquad/debug",1);
    traj= nh_.advertise<geometry_msgs::PoseArray>("traj", 5);
    ros::NodeHandle private_nh("~");
    private_nh.param<double>("robot_colision_x", rob_col_x, 0.0);
    private_nh.param<double>("robot_colision_y", rob_col_y, 0.0);
    private_nh.param<double>("robot_colision_z", rob_col_z, 1);
    private_nh.param<double>("explore_z_down_limit", explore_z_down_limit, 1);
}


/*
*
* step 1) update the octomap
* step 2) update robot position
* step 3) find the frontears: update the candidates for free space
*
*
*/
void octomap_search::find_status(){
    candidates.clear();
    if(!call_octomap())return;
    frontears();
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
    if (nh_.ok())
    { // skip when CTRL-C
    AbstractOcTree* tree=octomap_msgs::msgToMap(resp.map);
    octree = dynamic_cast<octomap::OcTree*>(tree);
    co_octomap++;
    if(co_octomap == 1)
    {
    octree_res = octree->getResolution();
    double x,y,z;
    octree->getMetricSize(x, y, z);

    }
    }
    return true;
}
// Free space
void octomap_search::frontears()
{
//defina area search
double octree_res = octree->getResolution();
int zd0 =z_limiter=explore_z_down_limit*(1/octree_res) ;

pose_occupied = false;
double dist=sqrt(pow(rob_col_x-goal_col_x,2)+pow(rob_col_y-goal_col_y,2));
dist=round(dist+2);

float axis=0;
if(rob_col_y>goal_col_y)
    axis=180;

for (float an=0+axis*deg; an<=(180+axis)*deg; an+=10*deg){
    for (float j=0;j<=dist;j+=0.5){
        double x=rob_col_x+j*cos(an);
        double y=rob_col_y+j*sin(an);
        oc_node = octree->search((double)x, (double)y, (double)zd0*octree_res);
           if(oc_node)
            {
                if(oc_node->getValue() <= 0)
                {
                    std::vector<float> points;
                    points.push_back(x);points.push_back(y);
                    points.push_back(1);points.push_back(an/deg);
                    candidates.push_back(points);
                }
            }

    }


}
return;


}

double octomap_search::cost_grid(std::vector<double> vec){
     double max=*std::max_element(vec.begin(),vec.end());
     double min=*std::min_element(vec.begin(),vec.end());
     return max;
 }
//cost computation
void octomap_search::computeCost(){
    /*
     * grid item is [n x 4] matrix=[x y z angle]
     * seperate items with respect to Angle (30 deg seems reasonable)
     * TODO: change the angle when robot_y<goal_y
     *
     */
    cost.clear();
    std::vector<double> dir_index_0;
    std::vector<double> dir_index_1;
    std::vector<double> dir_index_2;
    std::vector<double> dir_index_3;
    std::vector<double> dir_index_4;
    std::vector<double> dir_index_5;
    std::vector<double> dir_index_6;


    //compute distance for each of the grid
    foreach(vector<float>cell,candidates){
        if(cell.size()<4)ROS_ERROR("cost candidates size error");
        double dist=sqrt(pow(rob_col_x-cell.at(0),2)+pow(rob_col_y-cell.at(1),2));
        int index_ang=int(cell.at(3))/angle_sep;
        if(rob_col_y>goal_col_y)
            index_ang=6-int(cell.at(3))/angle_sep;
    //populate each of the group
        switch(abs(index_ang)){
            case 0:dir_index_0.push_back(dist); break;
            case 1:dir_index_1.push_back(dist); break;
            case 2:dir_index_2.push_back(dist); break;
            case 3:dir_index_3.push_back(dist); break;
            case 4:dir_index_4.push_back(dist); break;
            case 5:dir_index_5.push_back(dist); break;
            case 6:dir_index_6.push_back(dist); break;
        }
    }

    //compute cost for each of the grid
    foreach(vector<float>cell,candidates){
        if(cell.size()<4)ROS_ERROR("cost candidates size error");
        int index_ang=int(cell.at(3))/angle_sep;
        if(rob_col_y>goal_col_y)
            index_ang=6-int(cell.at(3))/angle_sep;
    //populate cost vector
        switch(abs(index_ang)){
            case 0:cost.push_back(cost_grid(dir_index_0)); break;
            case 1:cost.push_back(cost_grid(dir_index_1)); break;
            case 2:cost.push_back(cost_grid(dir_index_2)); break;
            case 3:cost.push_back(cost_grid(dir_index_3)); break;
            case 4:cost.push_back(cost_grid(dir_index_4)); break;
            case 5:cost.push_back(cost_grid(dir_index_5)); break;
            case 6:cost.push_back(cost_grid(dir_index_6)); break;
        }
    }

}

void octomap_search::updateRobot(float x, float y){
    rob_col_x=x;
    rob_col_y=y;
}

 void octomap_search::foundPath(vector<float>goal){

 //  1)report goal and robot state
//    ROS_INFO("robot (%f %f ) goal (%f %f)", rob_col_x, rob_col_y, goal.at(0),goal.at(1));
    goal_col_x=goal.at(0); goal_col_y=goal.at(1);
    std::stringstream debug_cntrle;
    debug_cntrle<<"From ("<<rob_col_x<<", "<<rob_col_y<<") to ("<<goal.at(0)<<", "<<goal.at(1)<<")";
    debugger(debug_cntrle.str());

 //  2) run dstar search algorithm with 0.5m resolution
    float scale=2.00;
    dstar = new Dstar();
    dstar->init(rob_col_x,rob_col_y, 1.5*goal_col_x,1.5*goal_col_y);
    dstar->updateStart(scale*rob_col_x, scale*rob_col_y);
    dstar->updateGoal(scale*goal.at(0),scale*goal.at(1));

 //  3) update cost function for the grid cells
    int canCount=0;
    bool sampleErr=false;
    path.clear();
    computeCost();
    if(cost.size()==candidates.size())
    foreach(vector<float>p,candidates){
        double cost_r=1/exp(cost.at(canCount));
       dstar->updateCell(scale*p.at(0),scale*p.at(1),cost_r );
       canCount++;
    }
    else
    {
        ROS_ERROR("Error! candidate %d != cost %d",cost.size(),candidates.size());

        return;
    }

 //  4) generate a path
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

 //  5) publish the path

    if(!candidates.empty()){

        viz->obstacleMap( candidates);
        if(!sampleErr)
        publishTraj();
    }
    else
        debugger("NO path found;");

    }


}

void octomap_search::publishTraj(){
    geometry_msgs::PoseArray trajArray;
         std::stringstream debug_cntrle;
        int i=0;
        std::vector<double>x_viz,y_viz;
        foreach (std::vector<float>p,path){
             i++;
             x_viz.push_back(p.at(0));
             y_viz.push_back(p.at(1));
             if(i>1){

                 geometry_msgs::Pose pose;
                 pose.position.x=p.at(0);
                 pose.position.y=p.at(1);
                 trajArray.poses.push_back(pose);
                 debug_cntrle<<"Target " << i-1<< ": ("<<p.at(0)<<", "<<p.at(1)<<")\n";
             }

        }
        traj.publish(trajArray);
        viz->pathViz(x_viz,y_viz);
        debugger(debug_cntrle.str());
}


/*
 * Utilitize 1) send message to gui
 * Utilitize 2) Enable point to point navigation
 *
 */

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


/*
 * obstacle avoidance function
 *  1) check obstacle to octree
 *  2) check obstacle around robot periphery
 */
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

void octomap_search::obstacle_avoid(double*A,double rx,double ry){
 A[0]=A[1]=0;
 bool obs=false;
    for (float an=0; an<=360*deg; an+=10*deg){

            double x=rx+cos(an);
            double y=ry+sin(an);
           if(octree_search(x,y)){
               obs=true;
               double bn=an/deg;
                ROS_INFO_STREAM("obstacle angle "<< bn);
                   if(   0<bn && bn<=45  )      A[0]+=1;
                   else if(  45<bn && bn<=135 )    A[1]+=1;
                   else if( 135<bn && bn<=225 )   A[0]+=-1;
                   else if( 225<bn && bn<=315 )   A[1]+=-1;
                   else if( 315<bn && bn<=360 )   A[0]+=1;

           }
    }
 if(obs){
    ROS_INFO_STREAM("obs "<<A[0]<<"\t"<<A[1]);
    A[0]/=abs(A[0]);
    A[1]/=abs(A[1]);}
 }


