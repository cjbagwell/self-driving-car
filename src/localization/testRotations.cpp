#include<iostream>
#include<ostream> 
#include "rotations.h"
#include<vector>

using namespace std;

int main(){
    vector<double> eAngles = {1, 2, 3};
    Quaternion t1 = eulerToQuaternion(eAngles);
    cout << t1 << endl;
}
