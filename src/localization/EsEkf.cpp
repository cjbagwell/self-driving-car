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
    this->qCov = diagmat(Row<double>({0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));
    this->lJac = Mat<double>(9,6, fill::zeros);
    lJac.submat(3,0,8,5) = eye(6,6);
}

EsEKF::EsEKF(State initialState, 
             vector<double> initialVariance)
             :
             EsEKF(initialState, Row<double>(initialVariance))
             {}

State EsEKF::runStep(ImuMeasurement m, vector<double> sensorVar){
    Row<double> sVar(sensorVar);
    return this->runStep(m, sVar);
}

State EsEKF::runStep(ImuMeasurement &m, Row<double> &sensorVar){
    // 1. Update state with IMU measurement (motion model)
    double dt = m.time - currState.time;
    Mat<double> rotMat = currState.rot.toRotMat();

    Col<double> pCheck = currState.pos + dt * currState.vel + ((dt*dt)/2) * (rotMat * m.accelerometer + G);
    Col<double> vCheck = currState.vel + dt * (rotMat * m.accelerometer + G);
    Quaternion  qChange= Quaternion(m.gyro * dt, true);
    Quaternion  qCheck = currState.rot * qChange; // order right for quat mult?
    // qCheck.normalize();

    // 2. Linearize the Motion Model with Jacobians
    Mat<double> fJac = eye(9,9);
    fJac.submat(0,3,2,5) = eye(3,3) * dt; // not sure about submatrix intexing...
    fJac.submat(3,6,5,8) = -skewSemetric(rotMat * m.accelerometer) * dt;
    
    // 3. Propogate Uncertainty
    Mat<double> var = diagmat(sensorVar) * dt*dt;
    qCov = qCov * var;
    pCov = fJac * pCov * fJac.t() + lJac * qCov * lJac.t();
    
    // Debugging Info
    cout << "======================================================================\n";
    cout << "t: " << m.time << "\t t: " << m.time << "\n";
    cout << "currPos:\n"<< currState.pos << "\n";
    cout << "currVel:\n"<< currState.vel << "\n";
    cout << "currQuat: "<< currState.rot << "\n";
    cout << "rotMat:\n" << rotMat   << "\n";
    cout << "pCheck:\n" << pCheck   << "\n";
    cout << "vCheck:\n" << vCheck   << "\n";
    cout << "qChange: " << qChange  << "\n";
    cout << "qCheck: "  << qCheck   << "\n";
    cout << "fJac:\n"   << fJac     << "\n";
    cout << "lJac:\n"   << lJac     << "\n";
    cout << "qCov:\n"   << qCov     << "\n";
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

State EsEKF::runStep(const State &measurement, const Row<double> &sensorVar){
    /** TODO: implement this method */
    return State();
}

// ========================================================================
// Pybind11 implementation
// ========================================================================

PYBIND11_MODULE(py_localization, handle){
        py::class_<EsEKF>(handle, "EsEkf")
            .def(py::init<State, vector<double>>())
            .def("run_step", py::overload_cast<ImuMeasurement, vector<double>>(&EsEKF::runStep))
            ;
        
        py::class_<State>(handle, "State")
            .def(py::init<>())
            .def(py::init<vector<double>, vector<double>, vector<double>,double,int>())
            .def("get_position", &State::getPosition)
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
}

int main(){
    cout << "Hello World!" << endl;
    return 1;
}