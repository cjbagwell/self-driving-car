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
#include<armadillo>

// project includes
// #include<pybind11/pybind11.h>
#include "Localization.h"
#include "ImuMeasurement.h"

using namespace std;
using namespace arma;
// namespace py = pybind11;

// ========================================================================
// Global Functions and Variables
// ========================================================================

const arma::Col<double> G({0.0, 0.0, -9.81});

// ========================================================================
// EsEKF Implementation
// ========================================================================

State EsEKF::runStep(ImuMeasurement &m, Row<double> &sensorVar){
    // 1. Update state with IMU measurement (motion model)
    double dt = m.time - currState.time;
    Mat<double> rotMat = currState.rot.toRotMat();
    Col<double> pCheck = currState.pos + dt * currState.vel + ((dt*dt)/2) * (rotMat * m.accelerometer + G);
    Col<double> vCheck = currState.vel + dt * (rotMat * m.accelerometer + G);
    Quaternion  qChange= Quaternion(m.gyro, true);
    Quaternion  qCheck = currState.rot * qChange; // order right for quat mult?
    
    // 2. Linearize the Motion Model with Jacobians
    Mat<double> fJac = eye(9,9);
    fJac.submat(0,3,2,5) = eye(3,3) * dt; // not sure about submatrix intexing...
    fJac.submat(3,6,5,8) = -skewSemetric(rotMat * m.accelerometer) * dt;
    
    Mat<double> lJac = arma::zeros(9,6);
    lJac.submat(3,0,8,5) = eye(6,6);

    // 3. Propogate Uncertainty
    pCov = fJac * pCov * fJac.t() + lJac * pCov * lJac.t();

    // 4. Update Filter State
    currState.pos = pCheck;
    currState.vel = vCheck;
    currState.rot = qCheck;
    currState.time= m.time;
    currState.frame= -1; /** TODO: need to figure out about frame from IMU measurement */
}


EsEKF::EsEKF(State initialState, Row<double> initialVariance):currState(initialState){
    this->currState = initialState;
    this->pCov = diagmat(initialVariance);
}

State EsEKF::runStep(const State &measurement, const Row<double> &sensorVar){
    /** TODO: implement this method */
}

