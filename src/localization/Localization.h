/**
 * @file Localization.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-05-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// std lib includes
#include<vector>
#include<list>
#include<iostream>
#include<ostream>
#include<armadillo>

// project lib includes
#include "State.h"

namespace localization{
    using namespace std;

    class NonlinearKF{
    private:
        State *currState, *prevState;
        arma::mat *K, *pCurr;

    public:
        /**
         * Constructor for Nonlinear Kalman Filters
         * @param initialState The starting location of the vehicle.
         */
        NonlinearKF(State initialState);

        /**
         * @returns the current state of the vehicle.
         */
        State* getCurrentState()const{return this->currState;}
        
        /**
         * updates the state of the vehicle based on a new measurement from the
         * Inertial Measurement Unit (IMU).
         * @param newImu the newest IMU measurement in the following Format
         *      TODO: add format for IMU measurement
         * @param sensorVar the variance of the IMU sensor
         *      TODO: add format for sensor variance
         * @returns the state of the vehicle after considering the new measurement
         */
        virtual State updateState(const arma::Row<double> &newImu, const arma::Row<double> &sensorVar);
        
        /**
         * updates the state of the vehicle based on a new measurement from the
         * the GNSS reciever of Lidar scanner.
         * @param measurement the measurement from the sensor
         * @param sensorVar the variance of the IMU sensor
         *      TODO: add format for IMU measurement
         * @returns the state of the vehicle after considering the new measurement
         */
        virtual State updateState(const State &measurement, const arma::Row<double> &sensorVar);
    };

    class EsEKF:NonlinearKF{
    public:
        /**
         * Constructor for an Error-state Extended Kalman Filter.
         * @param initialState The starting location of the vehicle.
         */ 
        EsEKF(State initialState);
    };

    class UnscentedKF:NonlinearKF{
    public:
        /**
         * Constructor for an Error-state Extended Kalman Filter.
         * @param initialState The starting location of the vehicle.
         */
        UnscentedKF(State initialState);
    };
}
#endif