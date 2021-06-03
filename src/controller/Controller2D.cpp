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
#include<list>
#include<tuple>
#include<iostream>
#include<iomanip>
#include<ostream>
#include<limits>
#include<algorithm>
#include<numeric>

// other lib includes
// #include<armadillo>

// project lib includes
#include<pybind11/pybind11.h>
#include<pybind11/stl.h>
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
    
    return atan(tan(angle));
}

Controller2D::~Controller2D(){

}

Commands Controller2D::runStep(State currState, Waypoint prevWaypoint, Waypoint currWaypoint, double dt){
    Commands newCommands;
    double requestedAcceleration = this->lonController.runStep(currState.speed, currWaypoint.getV(), dt);
    double requestedSteering = this->latController.runStep(currState, prevWaypoint, currWaypoint);
    // cout<< "current speed " << currState.speed << "\tcurrent targetSpeed " << currWaypoint.getV() << 
    //     "\trequested Acceleration " << requestedAcceleration << endl;


    newCommands.setSteeringAngleRate(requestedSteering);
    if(requestedAcceleration > 0.0){
        newCommands.setApp(max(requestedAcceleration, 1.0));
        newCommands.setBpp(0);
    }
    else{
        newCommands.setBpp(max(requestedAcceleration, -1.0));
        newCommands.setApp(0);
    }
    return newCommands;
}

double LongitudinalPIDController::runStep(double currentSpeed, double targetSpeed, double dt){
    double e = targetSpeed - currentSpeed;
    double de = 0.0;
    double ie = 0.0;
    if(errorBuffer.size() >= 2){
        de = (e - errorBuffer.front()) / dt;
        ie = accumulate(begin(errorBuffer), end(errorBuffer), e) * dt;
        
    }
    this->errorBuffer.push_front(e);

    double returnVal = kp*e + kd*de + ki*ie;
    // cout << "Lon Controller return Val (before max) " << returnVal << endl;
    return max(-1.0, min(returnVal, 1.0));
}

double LateralStanleyController::runStep(State currState, Waypoint prevWaypoint, Waypoint currWaypoint){
    // double a = tan(currWaypoint.getYaw());
    // double b = -1;
    // double c = currWaypoint.getY() - a * currWaypoint.getX();
    double dx = currState.x - currWaypoint.getX();
    double dy = currState.y - currWaypoint.getY();
    // arma::rowvec distVec = {dx, dy};
    // arma::rowvec frontAxleVec = {-cos(currState.yaw + PI/2), -sin(currState.yaw + PI/2)};

    double yawDesired = wrap2pi(currWaypoint.getYaw());   // desired heading (yaw)
    double yawError = wrap2pi(yawDesired - currState.yaw); // heading error
    double cte = -cos(currState.yaw + PI/2) * dx - sin(currState.yaw + PI/2) * dy; //cross track error
    double ctCorrection = atan2(kcte*cte, ks + currState.speed); //TODO: not sure about the minus velocity softening
    

    cout << fixed;
    cout << setprecision(4);
    cout << "lateralController \n" <<
            "yawDesired " << yawDesired << 
            "\tcurrYaw " << currState.yaw << 
            "\tyaw_error " << yawError << 
            "\ncte " << cte << 
            "\tctCor " << ctCorrection << 
            "\ncommand " << yawError + ctCorrection << 
            "\twrapped command " << wrap2pi(yawError + ctCorrection) <<
            "\n" << endl;
    return (yawError + ctCorrection)/PI;
}

PYBIND11_MODULE(py_controller, handle){
        py::class_<Waypoint>(handle, "Waypoint")
            .def(py::init<double, double, double, double>())
            .def("get_x", &Waypoint::getX)
            .def("get_y", &Waypoint::getY)
            .def("get_v", &Waypoint::getV)
            .def("get_yaw", &Waypoint::getYaw)
            ;
        py::class_<Commands>(handle, "Commands")
            .def(py::init<>())
            .def(py::init<double, double, double>())
            .def("get_app", &Commands::getApp)
            .def("get_bpp", &Commands::getBpp)
            .def("get_steering_angle_rate", &Commands::getSteeringAngleRate)
            ;
        py::class_<Controller2D>(handle, "Controller2D")
            .def(py::init<vector<Waypoint>, Commands>())
            .def(py::init<Commands, double, double, double, double, double>())
            .def("update_waypoints", &Controller2D::updateWaypoints)
            .def("run_step", &Controller2D::runStep)
            ;
        py::class_<State>(handle, "State")
            .def(py::init<>())
            .def(py::init<double,double,double,double,double,int>())
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