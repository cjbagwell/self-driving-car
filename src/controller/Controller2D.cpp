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
#include<armadillo>

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

Commands Controller2D::runStep(State currState, Waypoint currWaypoint, double dt){
    Commands newCommands;
    double requestedAcceleration = this->lonController.runStep(currState.getSpeed(), currWaypoint.getV(), dt);
    double requestedSteering = this->latController.runStep(currState, currWaypoint);

    newCommands.setSteeringAngleRate(requestedSteering);
    if(requestedAcceleration > 0.0){
        newCommands.setApp(requestedAcceleration);
        newCommands.setBpp(0);
    }
    else{
        newCommands.setBpp(requestedAcceleration);
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
    if(this->errorBuffer.size() > 10) this->errorBuffer.pop_back();

    double returnVal = kp*e + kd*de + ki*ie;
    return max(-1.0, min(returnVal, 1.0));
}

double LateralStanleyController::runStep(State currState,Waypoint currWaypoint){
    double dx = currState.pos[0] - currWaypoint.getX();
    double dy = currState.pos[1] - currWaypoint.getY();
    arma::rowvec distVec = {dx, dy};
    double yaw = currState.getYaw();
    arma::rowvec frontAxleVec = {-cos(yaw + PI/2), -sin(yaw + PI/2)};
    
    double yawDesired = wrap2pi(currWaypoint.getYaw()); // desired heading (yaw)
    double yawError   = wrap2pi(yawDesired - yaw);      // heading error
    double cte = arma::dot(distVec, frontAxleVec);      // cross track error
    double ctCorrection = atan2(kcte*cte, ks + currState.getSpeed());
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
}
