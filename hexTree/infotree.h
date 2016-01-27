#ifndef INFOTREE_H
#define INFOTREE_H
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include "display.h"
#include <iostream>
#define deg 0.0174533
#define AREASIZE 56
#define SAMPLESIZE 1


 template <class ForwardIterator>
        std::size_t min_element_index ( ForwardIterator first, ForwardIterator last )
      {
        ForwardIterator lowest = first;
        std::size_t index = 0;
        std::size_t i = 0;
        if (first==last) return index;
        while (++first!=last) {
          ++i;
          if (*first<*lowest) {
            lowest=first;
            index = i;
          }
        }
        return index;
      }


class display;

class infoTree
{
public:
    infoTree();
    struct node{
        int key;
        float location[2],cost,info;

    };
    struct infoNode{
        int item;
        node *parent;
        node *children[7];
    };
    std::vector<infoNode>logTree;


    int length();
    void addParent(float *);
    void treeToDPinput(float *value, float *weight);
    void parent_Measurement_Update(float,float,int);
    void current_tree();
    void currentchildren(float child[7][2]);
    void hamiltoPath(float child[7][2],float *result);
    int sequanceTOpath(int * , float *, float *);
    int sampleMeasurement(float mes_att[6],float cost_dist[6]);
private:
    infoNode *root;
    int serialKey;
    display *visualize;
    float children[7][2],robot[2];
    std::vector<int>taken;

protected:
    void addParentLocationPrivate(float *loc);
    void addChildrenPrivate(float *center);
    bool findMember(int,std::vector<int>);
    float distance(float A[2], float B[2]);
    void optimizeHamiltonPath(float *localpath, float *robot, float child[7][2] );


};

#endif // INFOTREE_H
