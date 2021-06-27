/**
 * @file Localization.cpp
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief 
 * TODO: some stuff here
 * @version 0.1
 * @date 2021-06-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
// std includes
#include<iostream>
#include<ostream>

// project includes
#include<armadillo>
#include<pybind11/pybind11.h>
#include<pybind11/stl.h>
#include "EsEkf.h"
#include "ImuMeasurement.h"
#include "rotations.h"
#include "GnssMeasurement.h"

using namespace std;
using namespace arma;
namespace py = pybind11;

// ========================================================================
// Global Functions and Variables
// ========================================================================

const arma::Col<double> G({0.0, 0.0, -9.81});

// ========================================================================
// EsEKF Implementation
// ========================================================================

EsEKF::EsEKF(State initialState, Row<double> initialVariance):currState(initialState){
    this->currState = initialState;
    this->pCov = diagmat(initialVariance);
    this->lJac = Mat<double>(9,6, fill::zeros);
    lJac.submat(3,0,8,5) = eye(6,6);
}

EsEKF::EsEKF(State initialState, 
             vector<double> initialVariance)
             :
             EsEKF(initialState, Row<double>(initialVariance))
             {}

State EsEKF::runStep(ImuMeasurement &m, vector<double> sensorVar){
    Row<double> sVar(sensorVar);
    return this->runStep(m, sVar);
}

State EsEKF::runStep(ImuMeasurement &m, Row<double> &sensorVar){
    // 1. Update state with IMU measurement (motion model)
    double dt = m.time - currState.time;
    Mat<double> rotMat = currState.rot.toRotMat();

    Col<double> pCheck = currState.pos + dt * currState.vel + ((dt*dt)/2) * (rotMat * m.accelerometer + G);
    Col<double> vCheck = currState.vel + dt * (rotMat * m.accelerometer + G);
    Quaternion  qChange= Quaternion(m.gyro * dt, false);
    Quaternion  qCheck = currState.rot * qChange; // order right for quat mult?
    // qCheck.normalize();

    // 2. Linearize the Motion Model with Jacobians
    Mat<double> fJac = eye(9,9);
    fJac.submat(0,3,2,5) = eye(3,3) * dt; // not sure about submatrix intexing...
    fJac.submat(3,6,5,8) = -skewSemetric(rotMat * m.accelerometer) * dt;
    
    // 3. Propogate Uncertainty
    Mat<double> qCov = diagmat(sensorVar * dt*dt);
    pCov = fJac * pCov * fJac.t() + lJac * qCov * lJac.t();
    
    // Debugging Info
    // cout << "======================================================================\n";
    // cout << "t: " << m.time << "\t t: " << m.time << "\n";
    // cout << "currPos:\n"<< currState.pos << "\n";
    // cout << "currVel:\n"<< currState.vel << "\n";
    // cout << "currQuat: "<< currState.rot << "\n";
    // cout << "rotMat:\n" << rotMat   << "\n";
    // cout << "pCheck:\n" << pCheck   << "\n";
    // cout << "vCheck:\n" << vCheck   << "\n";
    // cout << "qChange: " << qChange  << "\n";
    // cout << "qCheck: "  << qCheck   << "\n";
    // cout << "fJac:\n"   << fJac     << "\n";
    // cout << "lJac:\n"   << lJac     << "\n";
    // cout << "qCov:\n"   << qCov     << "\n";
    cout << "pCov:\n"   << pCov     << "\n";
    cout << endl;
    
    // 4. Update Filter State
    currState.pos = pCheck;
    currState.vel = vCheck;
    currState.rot = qCheck;
    currState.time= m.time;
    currState.frame= -1; /** TODO: need to figure out about frame from IMU measurement */
    return this->currState;
}



State EsEKF::runStep(GnssMeasurement &m, Row<double> &sensorVar){
    // Compute Jacobians
    Mat<double> hJac(3,9, fill::zeros);
    hJac.submat(0, 0, 2, 2) = eye(3,3);
    Mat<double> rJac = diagmat(sensorVar);
    
    // Compute Kalman Gain
    // cout << "hJac: \n" << hJac << "\n" << 
    // "pCov: \n" << pCov << "\n" << 
    // "rJac \n" << rJac << "\n" << endl;
    Mat<double> k = pCov * hJac.t() * inv(hJac * pCov * hJac.t() + rJac);

    // Compute Error State
    Col<double> yMeas = m.getLocation();
    Col<double> xErr = k * (yMeas - currState.pos);

    // Correct Predicted State
    currState.pos = currState.pos + xErr.subvec(0,2);
    currState.vel = currState.vel + xErr.subvec(3, 5);
    currState.rot = currState.rot * Quaternion(xErr.subvec(6, 8), false);

    // Update Covariance
    pCov = (eye(9,9) - k * hJac) * pCov;

    return currState;
}

// ========================================================================
// Pybind11 implementation
// ========================================================================

PYBIND11_MODULE(py_localization, handle){
        py::class_<EsEKF>(handle, "EsEkf")
            .def(py::init<State, vector<double>>())
            .def("run_step", py::overload_cast<ImuMeasurement&, vector<double>>(&EsEKF::runStep))
            .def("run_step", py::overload_cast<GnssMeasurement&, vector<double>>(&EsEKF::runStep))
            ;
        
        py::class_<State>(handle, "State")
            .def(py::init<>())
            .def(py::init<vector<double>, vector<double>, vector<double>,double,int>())
            .def("get_position", &State::getPosition)
            .def("get_velocity", &State::getVelocity)
            .def_readwrite("time", &State::time)
            .def_readwrite("rot", &State::rot)
            ;

        py::class_<ImuMeasurement>(handle, "ImuMeasurement")
            .def(py::init<>())
            .def(py::init<vector<double>, double, vector<double>, double>())
            .def("get_time", &ImuMeasurement::getTime)
            ;
        py::class_<Quaternion>(handle, "Quaternion")
            .def(py::init<vector<double>, bool>())
            .def("as_vector", &Quaternion::asVector)
            ;
        handle.def("euler_to_quat", &eulerToQuaternion);
        handle.def("quat_to_euler", &quaternionToEuler);

        py::class_<GnssMeasurement>(handle, "GnssMeasurement")
            .def(py::init<int, double, double, double, double>())
            // .def("get_location", &GnssMeasurement::getLocation)
            .def_readonly("frame", &GnssMeasurement::frame)
            .def_readonly("t", &GnssMeasurement::t)
            .def_readonly("alt", &GnssMeasurement::alt)
            .def_readonly("lat", &GnssMeasurement::lat)
            .def_readonly("lon", &GnssMeasurement::lon)
            ;
        handle.def("gnss_2_position", &GnssMeasurement2Position);
}

// int main(){
//     cout << "Hello World!" << endl;
//     return 1;
// }