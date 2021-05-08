/**
 * Localizer for Self Driving Car
 * Author:  C. Jordan Bagwell
 * Date:    5/6/2021
 * Description: TODO: some stuff here
 */

#include<vector>
#include<list>
#include<iostream>
#include<ostream>
#include "Controller2D.h"
// #include<armadillo>

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

namespace lcl{
    using namespace std;
    using namespace ctr;

    //TODO: temporary class that will be replaced with an armadillo matrix
    class Matrix{};

    class NonlinearKF{
    private:
        State *currState, *prevState;
        Matrix *K, *pCurr;

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
        virtual State updateState(const Matrix &newImu, Matrix sensorVar);
        
        /**
         * updates the state of the vehicle based on a new measurement from the
         * the GNSS reciever of Lidar scanner.
         * @param measurement the measurement from the sensor
         * @param sensorVar the variance of the IMU sensor
         *      TODO: add format for IMU measurement
         * @returns the state of the vehicle after considering the new measurement
         */
        virtual State updateState(const State &measurement, Matrix sensorVar);
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