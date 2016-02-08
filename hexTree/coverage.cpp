#include "coverage.h"

coverage::coverage()
{
    //manually defined area constraint
    area_constraint.area_x=area_constraint.area_y=7;
    area_constraint.sampling_length=0.5;
    // initialze local map
    local_map=new MST;
    reset_local_map_with_heading(UP);
    update_global_map(local_map,0);


}

int coverage::area_coverage_direction(float robot[2], int dir){

//    int mapIndex=get_map_index(robot);

////    int current_robot_direction=global_map.path[mapIndex].heading;
    int current_robot_direction=local_map->heading;
    int nex_direction;
    if(dir==-1)
     nex_direction=compliment_direction(current_robot_direction);
    else
        nex_direction=dir;
    //update global map information
    local_map->covered_index++;

    float max_index_for_path=area_constraint.area_y/area_constraint.sampling_length;
    local_map->coverage=max_index_for_path/(1+local_map->covered_index);
//    ROS_INFO_STREAM(" coverage "<<local_map->coverage);
    local_map->heading=nex_direction;

    if(local_map->coverage<1){
        nex_direction=RIGHT;
        if(local_map->dir==UP)
            reset_local_map_with_heading( DOWN );
        else
             reset_local_map_with_heading(UP);
    }

    return nex_direction;

}

void coverage::reset_local_map_with_heading(direction x ){
    local_map->coverage=local_map->covered_index=local_map->heading=0;
    if(x==DOWN)
        local_map->heading=3;
    local_map->dir=x;
 }


int coverage::compliment_direction(int dir){
    switch (dir) {
        case 0:return 5;
        case 1:return 4;
        case 2:return 3;
        case 3:return 2;
        case 4:return 1;
        case 5:return 0;
    }

}

void coverage::update_global_map(MST *node,int index){
//    global_map.index=index;
//    global_map.path[index]=node;
}

int coverage::get_map_index(float robot[2]){

    float power=0;
    for(int j=0;j<2;j++)
        power+=pow(robot[j],2);
    float distance=sqrt(power);
    return distance/(4*area_constraint.sampling_length);
}
