/**
 * @file EsEkf.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-05-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef ES_EKF_H
#define ES_EKF_H

// std lib includes
#include<vector>
#include<list>
#include<iostream>
#include<ostream>
#include<armadillo>

// project lib includes
#include "State.h"
#include "ImuMeasurement.h"
#include "GnssMeasurement.h"

class EsEKF{
private:
    State currState, prevState;
    arma::Mat<double> pCov, lJac;

public:
    /**
     * Constructor for Nonlinear Kalman Filters
     * @param initialState The starting location of the vehicle.
     */
    EsEKF(State initialState, arma::Row<double> initialVariance);

    EsEKF(State initialState, std::vector<double> initialVariance);

    /**
     * @returns the current state of the vehicle.
     */
    State getCurrentState()const{return this->currState;}
    
    /**
     * @brief a simple way to convert from a vector<double> into Row<double>.
     * This is used for the interface with python.  Note this may be temporary,
     * but I am doing this with the expectation the conversion from list to vector
     * and from vector to arma::Row is well efficient.
     * 
     * @param m 
     * @param sensorVar 
     * @return State 
     */
    State runStep(ImuMeasurement &m, std::vector<double> sensorVar);

    /**
     * updates the state of the vehicle based on a new measurement from the
     * Inertial Measurement Unit (IMU).
     * @param newImu the newest IMU measurement in the following Format
     *      TODO: add format for IMU measurement
     * @param sensorVar the variance of the IMU sensor
     *      TODO: add format for sensor variance
     * @returns the state of the vehicle after considering the new measurement
     */
    State runStep(ImuMeasurement &newImu, arma::Row<double> &sensorVar);
    
    State runStep(GnssMeasurement &m, std::vector<double> sensorVar){
        arma::Row<double> sVar(sensorVar);
    return this->runStep(m, sVar);
}

    /**
     * updates the state of the vehicle based on a new measurement from the
     * the GNSS reciever of Lidar scanner.
     * @param measurement the measurement from the sensor
     * @param sensorVar the variance of the IMU sensor
     *      TODO: add format for Gnss measurement
     * @returns the state of the vehicle after considering the new measurement
     */
    State runStep(GnssMeasurement &m, arma::Row<double> &sensorVar);
};
#endif /* ES_EKF_H */