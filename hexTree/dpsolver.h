#ifndef DPSOLVER_H
#define DPSOLVER_H
#include <fstream>
#include <iostream>
#include <algorithm>
#include <boost/dynamic_bitset.hpp>
#include <ros/ros.h>

using namespace std;
//Container for item
class Item {
    int weight;
    int value;
    float ratio;
    public:
        void setup (int, int);
        int get_value() const;
        int get_weight() const;
        float get_ratio() const;
};


//Comparison for item class

inline bool operator<(const Item& x, const Item& y) {
    return x.get_weight() < y.get_weight();
}


class dpsolver
{
public:
    dpsolver();
    void input(int item_size, float *weight, float *value,int cap, int *resultIndex );
private:
    Item* items ;
    float** table ;
    int capacity;
    int num_items;
protected:
    void findSolution(int *indexSet);
    void copytable(float** table, int y);
    void printable(float** table, int x, int y);
};

#endif // DPSOLVER_H
