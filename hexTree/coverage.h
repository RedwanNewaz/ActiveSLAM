#ifndef COVERAGE_H
#define COVERAGE_H
#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include "hotspot.h"


class coverage
{
public:
    coverage();
       int area_coverage_direction(float robot[2], int dir=-1);
private:
    enum direction{
        UP,
        DOWN,
        LEFT=4,
        RIGHT=1
    };

    struct MST{
        int heading;
       int covered_index;
       enum direction dir;
       float coverage;
    };

    struct MAP{
        int index;
        MST path[10];
    }global_map;

    struct constraint{
        float area_x,area_y;
        float sampling_length;
    }area_constraint;

    MST *local_map;

protected:
    int get_map_index(float robot[2]);
    int compliment_direction(int);
    void update_global_map(MST *node,int index);
    void reset_local_map_with_heading(direction x );

};


#endif // COVERAGE_H
