#include "actuator.h"
#include "Dstar.h"
#include "tum_ardrone/filter_state.h"
#include <QFile>
actuator::actuator()
{
    mapExist=false;
    dronepose_pub	   = nh_.advertise<tum_ardrone::filter_state>(nh_.resolveName("ardrone/predictedPose"),1);

    trackerFile="/home/redwan/Desktop/data/trackerFile.txt";
    QString OtrackerFile="/home/redwan/Desktop/data/trackerFile_";
    for(int i=1;QFile(trackerFile).exists();i++)
         trackerFile=OtrackerFile+QString::number(i)+".txt";
}

//RELATED TO PATH PLANNING
 list<state> actuator::run(TODPOSITION robo ,TODPOSITION goal){

  cout<< "robot{"<<robo.x<<" "<<robo.y<<"}\t goal{"<< goal.x<<" "<<goal.y<<"}"<<endl;
  robo=TODPOSITION{robo.x*PLANNINGSCALE,robo.y*PLANNINGSCALE};
  goal=TODPOSITION{goal.x*PLANNINGSCALE,goal.y*PLANNINGSCALE};

  dstar = new Dstar();
    dstar->init(robo.x,robo.y, goal.x,goal.y);
    if(mapExist)
        searchSpace();


//        update goal position if required
//        dstar->updateGoal(mG.x(),mG.y());
        dstar->replan();
         foreach(state p, dstar->getPath())
             mypath.push_back( state{p.x/PLANNINGSCALE,p.y/PLANNINGSCALE});


        //may be print is required to understand
//        printPath();
        return mypath;
}

 void actuator::map(){
    mapExist=true;
}

 void actuator::searchSpace(){

}

 void actuator::printPath(){
    foreach (state path,mypath){
        cout<< "path{"<< path.x<<" "<<path.y<<"}"<<endl;
    }
}

 void actuator::trackerState(vector<double>state, int tr){

       QFile file(trackerFile);
       file.open(QIODevice::Append | QIODevice::Text);
       if(file.isOpen()){
         QTextStream outStream(&file);
         outStream<<getMS()<<"\t";
         for(int i=0;i<8;i++)
            outStream<<state.at(i)<<"\t";
         outStream<<tr<<"\n";


     }
 }

// RELATED TO STATE PUBLISHING

 void actuator::statePublish(vector<double> state){
    tum_ardrone::filter_state s;

    s.header.stamp = ros::Time().now();
//    POSITION
    s.x=state.at(0);
    s.y=state.at(1);
    s.z=state.at(2);
//    VELOCITY
    s.dx=state.at(3);
    s.dy=state.at(4);
    s.dz=state.at(5);

//    ORIENTATION
    s.roll=state.at(6);
    s.pitch=state.at(7);
    s.yaw=state.at(8);
    s.dyaw=state.at(9);

//    UTILITIES
    s.batteryPercent=state.at(10);
    s.droneState=state.at(11);
    s.ptamState=5;//PTAM TOOK KF

    dronepose_pub.publish(s);

 }

