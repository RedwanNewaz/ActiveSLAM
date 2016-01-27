#include "infotree.h"
#include <ros/ros.h>
#include "display.h"
using namespace std;
infoTree::infoTree()
{
    root=new infoNode;
    serialKey=0;
    visualize=new display();

}

void infoTree::addParent(float *loc)
{
      root->parent=new node;
      root->parent->key=serialKey;
      root->parent->location[0]=loc[0];
      root->parent->location[1]=loc[1];
      root->parent->cost=NULL;
      root->parent->info=NULL;

      if(serialKey==0)
      {
          robot[0]=loc[0];
          robot[1]=loc[1];
      }


      serialKey++;
      addChildrenPrivate(loc);

}

void infoTree::parent_Measurement_Update(float cost,float info,int direction)
{
       root->parent->cost=cost;
       root->parent->info=info;
       root->item=direction;
       logTree.push_back(*root);

}

//main operation
void infoTree::addChildrenPrivate(float *center){

    int count=0; float R=SAMPLESIZE;
    for(float theta=0;theta<360;theta+=60){
       root->children[count]=new node;
       root->children[count]->location[0]= children[count][0]=center[0]+R*sin(theta*deg);
       root->children[count]->location[1]= children[count][1]=center[1]+R*cos(theta*deg);
       count++;
    }
    float localPath[6];
    hamiltoPath(children,localPath);
    optimizeHamiltonPath(localPath,robot,children);
    visualize->hexSample(children,localPath);
}

void infoTree::current_tree(){
    cout<<"Tree size "<<serialKey<<endl
       <<"parent location \t"<< root->parent->location[0]<<"," <<root->parent->location[1]<<endl
       <<"children location"<<endl;

       for (int count=0;count<6;count++)
           printf("(%0.3f, %0.3f)\n",root->children[count]->location[0],
                   root->children[count]->location[1]);

}

void infoTree::currentchildren(float child[7][2]){
    for (int i=0;i<7;i++){
        child[i][0]=children[i][0];
        child[i][1]=children[i][1];
    }

}

int infoTree::sampleMeasurement(float mes_att[6],float cost_dist[6]){
    int maxEdgeIndex=0;
    float minimum=1000;
    for (int i=0;i<6;i++){
        root->children[i]->cost=cost_dist[i];
        root->children[i]->info=mes_att[i];
        if(abs(mes_att[i])<minimum){
            minimum=mes_att[i];
            maxEdgeIndex=i;
        }

    }
    parent_Measurement_Update(cost_dist[maxEdgeIndex],minimum,maxEdgeIndex);
    return maxEdgeIndex;

}

void infoTree::hamiltoPath(float nei[7][2],float *result){

    for(int i=0;i<6;i++){
        int indexNei=nei[i][0]+AREASIZE*nei[i][1];
        if(findMember(indexNei,taken)){
            result[i]=-1;
        }
        else
        {
            result[i]=i;
            float B[2]={nei[i][0],nei[i][1]};
            taken.push_back(indexNei);
        }
    }

}

float infoTree::distance(float A[2], float B[2]){
    return sqrt(pow(A[0]-B[0],2)+pow(A[1]-B[1],2));
}

bool infoTree::findMember(int candidate,std::vector<int>list)
{
    if(list.empty())return 0;

    for(int i=0;i<list.size();i++)
            if(list.at(i)==candidate)
                return true;

   return false;


}

int infoTree::length(){
//    return parentInfo.size();
    return logTree.size();
}

void infoTree::treeToDPinput(float *value, float *weight){
    Eigen::VectorXf infoPar(logTree.size()),costPar(logTree.size()),
            dirChange(logTree.size());
    int intialDir=-1;
    for (int i=0;i<logTree.size();i++){
        infoNode x=logTree.at(i);
        infoPar(i)=x.parent->info;
        costPar(i)=x.parent->cost;
        if(intialDir!=x.item){
        dirChange(i)=1;
        intialDir=x.item;
        }
        else
            dirChange(i)=0;
    }
    for (int i=0;i<logTree.size();i++){
        weight[i]= 1000*dirChange(i)*infoPar(i)/infoPar.norm();
        value[i]=1000*costPar(i)/costPar.norm();
    }

}

int infoTree::sequanceTOpath(int * sequance, float *X, float *Y){

    int count =0;
    for(int i=0;i<logTree.size();i++){
        if(sequance[i]==1)continue;
        infoNode nd=logTree.at(i);
        X[count]=nd.parent->location[0];
        Y[count]=nd.parent->location[1];
        count++;
    }
    infoNode nd=logTree.at(logTree.size()-1);
    X[count]=nd.parent->location[0];
    Y[count]=nd.parent->location[1];

    visualize->optimalPath(X,Y,count+1);
    return count;
}

void infoTree::optimizeHamiltonPath(float *pathIndex, float *robo, float nei[7][2] ){

    float alist[7][2];
    //cost and index container
    int count=0;
    for (int i=0;i<6;i++){
        if (pathIndex[i]==-1)continue;
        int j=pathIndex[i];
        float cost=sqrt(pow(robo[0]-nei[j][0],2)+
                        pow(robo[1]-nei[j][1],2));

         alist[count][0]=cost;
         alist[count][1]=i;
         count++;
    }

    //bubule sort
      for (int c = 0 ; c < count-1; c++)
          for (int d = 0 ; d < count - c - 1; d++)
          {

              if (alist[d][0] > alist[d+1][0])
              {
                /* Swapping */

                float t   = alist[d][0];
                alist[d][0]   = alist[d+1][0];
                alist[d+1][0] = t;

                float t1      = alist[d][1];
                alist[d][1]   = alist[d+1][1];
                alist[d+1][1] = t1;
              }
          }

    // reconvert the solution
      int jcount=0;
      for (int i=0;i<6;i++)
      {
          if (pathIndex[i]==-1)continue;
          pathIndex[i]=alist[jcount][1];
          jcount++;
      }
}


