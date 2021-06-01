/**
 * @file Controller2D.cpp
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-06-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// std lib includes
#include<math.h>
#include<vector>
#include<tuple>
#include<iostream>
#include<ostream>
#include<limits>

// project lib includes
#include<pybind11/pybind11.h>
#include "Controller2D.h"

using namespace std;
using namespace controller;
namespace py = pybind11;

const double GAIN_P = 1.0;  //Proportional error gain
const double GAIN_D = 0.0;  //Derivative error gain
const double GAIN_I = 0.25; //Integral error gain
const double GAIN_CTE = 1; // crossTrack error gain
const double VELOCITY_SOFTENING = 0.2;
const double PI = 3.14159;

/**Returns the Euclidean norm distance of the two args*/
inline double normDistance(pair<double, double> first, pair<double,double> second){
    double dx = first.first - second.first;
    double dy = first.second - second.second;
    return sqrt(dx*dx + dy*dy);
}

void printCommands(Commands c){
    cout << "current commands: APP=" << c.getApp() << "\tBPP=" << c.getBpp() << "\tangleRate=" << c.getSteeringAngleRate() << endl;
}

/**Wraps input to -PI/2 -> +PI/2 */
double wrap2pi(double angle){
    if(angle > PI/2){
        angle -= PI; 
    }
    else if(angle < PI/2){
        angle += PI;
    }
    return angle;
}

Controller2D::~Controller2D(){

}

void Controller2D::updateState(const State &egoState){
    this->prevState = this->currState;
    this->currState = egoState;
}


bool Controller2D::updateCommands(){
    // calculate APP and BPP Requirements
    double dt = currState.time - prevState.time;
    updateDesiredSpeed();
    double vError = this->vDesired - currState.speed;
    double vErrorPrev = this->vDesiredPrev - prevState.speed;
    double vErrorDot = (vError - vErrorPrev) / dt;
    this->vErrorInt += dt* vError;
    double aRequested = GAIN_P * vError + GAIN_I * vErrorInt + GAIN_D * vErrorDot;
    double kAPP = 1, kBPP = 1;
    if(aRequested >= 0){
        currCommands.setBpp(0);
        currCommands.setApp(max(100.0, kAPP * aRequested));
    }
    else{
        currCommands.setApp(0);
        currCommands.setBpp(max(100.0, kBPP * -aRequested));
    }

    // Calculate Steering Requirement
    double a = waypoints[0].getY() - waypoints[1].getY();
    double b = waypoints[1].getX() - waypoints[0].getX();
    double c = waypoints[0].getX()*waypoints[1].getY() - waypoints[1].getX()*waypoints[0].getY();
    
    double yawDesired = atan2(-a, b);   // desired heading (yaw)
    double yawError = wrap2pi(yawDesired - currState.yaw); // heading error
    double cte = (a*currState.x + b*currState.y + c) / sqrt(a*a + b*b); //cross track error
    double ctCorrection = atan2(GAIN_CTE * cte, -VELOCITY_SOFTENING + currState.speed); //TODO: not sure about the minus velocity softening
    currCommands.setSteeringAngleRate(yawError + ctCorrection);
    printCommands(this->currCommands);
    return true;
}

void Controller2D::updateDesiredSpeed(){
    double minDist = numeric_limits<double>::max();
    this->vDesiredPrev = this->vDesired;
    for(auto w : waypoints){
        double currDist = normDistance(make_pair(w.getX(), w.getY()), make_pair(currState.x, currState.y));
        if(currDist < minDist){
            minDist = currDist;
            this->vDesired = w.getV();
        }
    }
}

PYBIND11_MODULE(py_controller, handle){
        py::class_<Waypoint>(handle, "Waypoint")
            .def(py::init<double, double, double>())
            .def("get_x", &Waypoint::getX)
            .def("get_y", &Waypoint::getY)
            .def("get_v", &Waypoint::getV)
            ;
}

// int main(){
//     vector<Waypoint> testWaypoints;
//     Waypoint w1, w2, w3, w4, w5;
//     w1.v = 1;
//     w2.x = 1;
//     w2.v = 2;
//     w3.x = 2;
//     w3.v = 3;
//     w4.x = 3;
//     w4.v = 4;
//     w5.x = 4;
//     w5.v = 5;
//     testWaypoints.push_back(w1);
//     testWaypoints.push_back(w2);
//     testWaypoints.push_back(w3);
//     testWaypoints.push_back(w4);
//     testWaypoints.push_back(w5);

//     vector<State> testStates;
//     for(int i = 0; i < 5; i++){
//         State testState;
//         testState.time = i;
//         testState.x = i;
//         testState.yaw = 0;
//         testState.speed = i;
//         testState.frame = i;
//         testStates.push_back(testState);
//     }

//     Controller2D testController(testWaypoints);
//     for(int i = 0; i < 5; i++){
//         cout << "iteration :  " << i << endl;;
//         cout << "currState: x=" << testStates[i].x << "\ty=" << testStates[i].y << "\tyaw=" << testStates[i].yaw << endl;
//         testController.updateState(testStates[i]);
//         testController.updateCommands();
//         testController.getCommands();
//     }
// }