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

State EsEKF::runStep(const Row<double> &newImu, const Row<double> &sensorVar){
    /** TODO: implement this method */
    
}


EsEKF::EsEKF(State initialState, Row<double> initialVariance):currState(initialState){
    /** TODO: implement this method */   
}

State EsEKF::runStep(const State &measurement, const Row<double> &sensorVar){
    /** TODO: implement this method */
}

