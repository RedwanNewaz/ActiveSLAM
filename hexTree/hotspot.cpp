#include "hotspot.h"

hotspot::hotspot()
{
    step=0;
    tree=new infoTree;
    robot_client=nh_.serviceClient<active_slam::obstacle>("localization");
    traj= nh_.advertise<geometry_msgs::PoseArray>("traj", 10);
    service = nh_.advertiseService("xbee/measurements",&hotspot::sensor_reading,this);
}



void hotspot::findHotspot(float robot[2],float goal[2]){


    active_slam::obstacle robo;
    if (robot_client.call(robo))
    {
        robot[0]=robo.response.x;
        robot[1]=robo.response.y;
        ROS_WARN("Robot position is updated");
    }
    else
        ROS_ERROR("State Estimator is not active...");


    for(int i=0;i<2;i++){
        uav[i]=initialRobotPosition[i]=robot[i];
        target[i]=goal[i];
    }

    //initialization
    tree->addParent(uav);

    tree->currentchildren(HexSamples);
    sleep(1);
    sampling_path_publish();

    ros::Rate r(10);
    while (ros::ok()){
        ros::spinOnce();
         r.sleep();

    }

//    //simulation
//    simulation();



//    // find optimal path
//    optimal_path_publish();

}

void hotspot::simulation()
{

        // compute cost and info
      MatrixXf nei(6,2),subgoal(6,2);
      VectorXf MeasurementAttribute(6);

      nei=array2Mat(HexSamples);
      subgoal=repeatMat(target);
      MeasurementAttribute=distance(subgoal,nei);
      updateMeasurement(MeasurementAttribute.data());

     // termination condition
    float near=sqrt(pow(target[0]-uav[0],2)+pow(target[1]-uav[1],2));
     ROS_INFO_STREAM("step:= "<<step<< " distance:= "<<near);
     step++;

     sleep(1);
     if(near>1)simulation();

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

//-------------------------TODay

MatrixXf hotspot::repeatMat(float *array){
    MatrixXf mat(6,2);
     for (int i=0;i<6;i++)
       for (int j=0;j<2;j++)
            mat(i,j)=array[j];

     return mat;


}

MatrixXf hotspot:: array2Mat(float array[7][2]){
    MatrixXf mat(6,2);
    for (int i=0;i<6;i++)
      for (int j=0;j<2;j++)
          mat(i,j)=array[i][j];
    return mat;

}

void hotspot::updateMeasurement(float attribute[6])
{

    MatrixXf nei(6,2),iniPos(6,2);
    iniPos=repeatMat(initialRobotPosition);
    nei=array2Mat(HexSamples);

    VectorXf BaseCost(6);
    BaseCost=distance( iniPos , nei);

    //    update Tree and next sample location
    int local_best=tree->sampleMeasurement(attribute,BaseCost.data());
    updateParentLocation(uav,local_best);
    tree->addParent(uav);
    tree->currentchildren(HexSamples);
    ROS_INFO("updated uav (%f, %f)",uav[0],uav[1]);
    sampling_path_publish();



}

void hotspot::sampling_path_publish(){

    float X[6],Y[6];
    int pathLength=tree->optimize_sample_path(X, Y);
    geometry_msgs::PoseArray trajArray;

    for (int i=0;i<pathLength;i++)
    {
      geometry_msgs::Pose pose;
      pose.position.x=X[i];
      pose.position.y=Y[i];
      trajArray.poses.push_back(pose);
    }
      traj.publish(trajArray);


 }

void hotspot::optimal_path_publish(){

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
        key=cur;
    }

//    convert sequence to path
    float pathx[item],pathy[item];
    int path_length=tree->sequanceTOpath(solutionIndex,pathx,pathy);

    //publish path
    geometry_msgs::PoseArray trajArray;

    for(int i=0;i<path_length;i++)
    {
     geometry_msgs::Pose pose;
     pose.position.x=pathx[i];
     pose.position.y=pathy[i];
     trajArray.poses.push_back(pose);
    }
     traj.publish(trajArray);


 }

void hotspot::pathSeqToMes(float reading[6],int n){
    float hex_mes[6];int sequence[6];
    for(int i=0;i<6;i++)
        hex_mes[i]=1000;
    tree->pathSequence(sequence);
    for(int j=0;j<n;j++){
        if(sequence[j]==-1){
         ROS_ERROR("measurement update error");
         continue;
        }

        hex_mes[sequence[j] ]=reading[j];
    }
    updateMeasurement(hex_mes);


}

bool hotspot::sensor_reading(active_slam::sensor::Request  &req,
         active_slam::sensor::Response &res)
{
    float mes[6];
    for(int i=0;i<req.count;i++)
    {
        ROS_INFO_STREAM("measurements for path "<<req.reading[i]);
        mes[i]=req.reading[i];
    }
    pathSeqToMes(mes,req.count);
    res.result=true;
    return true;
}
