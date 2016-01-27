#include "dpsolver.h"

dpsolver::dpsolver()
{


}

void dpsolver::input(int item_size, float *weight, float *value,int cap, int *resultIndex ){
        ROS_INFO("DP SOLVER WORKING");
        items = new Item[item_size + 1];
        for (int i = 0; i < item_size; i ++){
              items[i+1].setup(value[i], weight[i]);
          }
        num_items=item_size;
        capacity=cap;
        table = new float*[2];
        for (int i = 0; i< 2; i++){
            table[i] = new float[capacity+1];
        }
        findSolution(resultIndex);

}

void dpsolver::findSolution(int *indexSet){



    std::vector<boost::dynamic_bitset<> > items_used(capacity+1, boost::dynamic_bitset<>(num_items+1)); //Bit array
    std::vector<boost::dynamic_bitset<> > items_used2(capacity+1, boost::dynamic_bitset<>(num_items+1)); //Bit array


     int a,b; //temps

     for (int i = 1; i<num_items+1; i++){
         for (int j = 1; j<capacity+1; j++){
             if (items[i].get_weight() <= j){
                 a = table[0][j];
                 b = table[0][j-items[i].get_weight()] + items[i].get_value();
                 if (b>a){
                     //cout << "A " << a << "B " << b << endl;
                     //cout << "Before " << items_used[j] << endl;
                     items_used[j] = items_used2[j-items[i].get_weight()];
                     //cout << "After " << items_used[j] << endl;
                     items_used[j][i] = 1;
                     //cout << "Set j " << j << " i " << i << ' ' << items_used[j] << endl;
                     table[1][j] =b;
                 } else{
                     table[1][j] =a;
                     items_used[j] = items_used2[j-1];
                 }
                 //table[1][j] = max(table[0][j],
                 //    table[0][j-items[i].get_weight()] + items[i].get_value());
             }else{
                 table[1][j] = table[0][j];
                 items_used[j] = items_used2[j-1];
                /* for (int n = 1; n<i;n++){
                         items_used[j][n] = items_used[j-1][n-1];
                 }*/
                 //items_used[j] = items_used[j-1];
                 //items_used[i-1][j-1]
             }
         }
         items_used2 = items_used;
         copytable(table, capacity+1);
     }
     //Print answer
//     cout << table[1][capacity]<< " " << endl;

     for (int i = 1; i<num_items; i++){
         cout << items_used[capacity][i] << ' ';
         indexSet[i-1]=items_used[capacity][i];
     }

     cout << items_used[capacity][num_items] << endl;
    indexSet[num_items-1]=items_used[capacity][num_items];


}

void dpsolver::copytable(float** table, int y){
    for (int i = 0; i< y; i++){
        table[0][i] = table[1][i];
    }
}

void dpsolver::printable(float** table, int x, int y){
    for (int i = 0; i< x; i++){
        for (int j =0; j<y; j++){
            cout << table[x][y] << " ";
        }
        cout << endl;
    }
}

void Item::setup (int v, int w){
    weight = w;
    value = v;
    ratio = (float)value/weight;
}

int Item::get_value () const{
    return value;
}

int Item::get_weight () const{
    return weight;
}

float Item::get_ratio () const{
    return ratio;
}

