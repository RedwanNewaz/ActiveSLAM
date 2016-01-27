#include "hotspot.h"

hotspot::hotspot()
{
    step=0;
    tree=new infoTree;
}
void hotspot::findHotspot(float robot[2],float goal[2]){

    for(int i=0;i<2;i++){
        uav[i]=initialRobotPosition[i]=robot[i];
        target[i]=goal[i];
    }
    enableSimulation=true;
    while(enableSimulation){
        simulation();
        sleep(1);
    }

    // find optimal path

    // tree length
    int item=tree->length();
    float value[item], weight[item];

    tree->treeToDPinput(value,weight);
    // populate value and item
    int solutionIndex[item];

    int key=100000;
    for(int pathLength=50;pathLength<168;pathLength+=15){
    dp=new dpsolver;
    dp->input(item,weight,value,pathLength,solutionIndex );

    //find optimal sequence
    int cur=0;
    for (int j=0;j<item;j++)
        cur+=solutionIndex[j]*(j+1);
    if(cur==key)
        break;
    else
        key=cur;
    }

//    convert sequence to path
    float pathx[item],pathy[item];
    tree->sequanceTOpath(solutionIndex,pathx,pathy);
}

void hotspot::simulation()
{

    // determine hexagonal sample locations
    tree->addParent(uav);
    float childern[7][2];
    tree->currentchildren(childern);
    MatrixXf nei(6,2),subgoal(6,2),iniPos(6,2);
    for (int i=0;i<6;i++)
        for (int j=0;j<2;j++){
            nei(i,j)=childern[i][j];
            subgoal(i,j)=target[j];
            iniPos(i,j)=initialRobotPosition[j];
        }

    // compute cost and info
    VectorXf BaseCost(6),MeasurementAttribute(6);
    BaseCost=distance( iniPos , nei);
    MeasurementAttribute=distance(subgoal,nei);

    //    update Tree and next sample location
    int local_best=tree->sampleMeasurement(MeasurementAttribute.data(),BaseCost.data());
    updateParentLocation(uav,local_best);

    // termination condition
    float near=sqrt(pow(target[0]-uav[0],2)+pow(target[1]-uav[1],2));
    if (near<1)
        enableSimulation=0;
    else
        ROS_INFO_STREAM("step:= "<<step<< " distance:= "<<near);

    step++;


}

void hotspot::updateParentLocation(float *robot,int candidate){

    float xx[4],yy[4];
    switch(candidate){
    case 0: readChildren(xx,yy,5,0,1,2) ;break;
    case 1: readChildren(xx,yy,0,1,2,3) ;break;
    case 2: readChildren(xx,yy,1,2,3,4) ;break;
    case 3: readChildren(xx,yy,2,3,4,5) ;break;
    case 4: readChildren(xx,yy,3,4,5,0) ;break;
    case 5: readChildren(xx,yy,4,5,1,0) ;break;
    }

//    for(int i=0;i<4;i++)
//        ROS_INFO("xx(%f) yy(%f)",xx[i],yy[i]);
    lineIntesection(robot,xx,yy);

}

void hotspot::readChildren(float* xx,float* yy,int i,int j,int k,int l){
    float child[7][2];
    tree->currentchildren(child);
    xx[0]=child[i][0];
    xx[1]=child[j][0];
    xx[2]=child[k][0];
    xx[3]=child[l][0];

    yy[0]=child[i][1];
    yy[1]=child[j][1];
    yy[2]=child[k][1];
    yy[3]=child[l][1];

}

void hotspot::lineIntesection(float *center, float xx[4], float yy[4]){

    float x12 = xx[0] - xx[1];
    float x34 = xx[2] - xx[3];
    float y12 = yy[0] - yy[1];
    float y34 = yy[2] - yy[3];

    float c = x12 * y34 - y12 * x34;
    float x,y;
    if (fabs(c) < 0.01){
            x=0;y=0;
    }
    else{
       float a = xx[0] * yy[1] - yy[0] * xx[1];
       float b = xx[2] * yy[3] - yy[2] * xx[3];
       x = (a * x34 - b * x12) / c;
       y = (a * y34 - b * y12) / c;
    }
    center[0]=x;center[1]=y;
}

Matrix<float,Dynamic,1> hotspot::distance(MatrixXf A,MatrixXf B,int size){

        MatrixXf D(size,2),D_pow(size,2);
        D=A-B;
        for (int i=0;i<size;i++)
            for (int j=0;j<2;j++)
                D_pow(i,j)=pow(D(i,j),2);
         return D_pow.rowwise().sum();

}
