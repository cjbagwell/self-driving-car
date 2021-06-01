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
#include "../controller/Controller2D.h"
#include<armadillo>

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

namespace localization{
    using namespace std;
    using namespace controller;

    /**
     * @brief A State of a vehicle to be used for a self-driving car.
     * @param x The x location of the vehicle[m].
     * @param y The y location of the vehicle [m].
     * @param yaw The yaw of the vehicle [rad].
     * @param speed The speed of the vehicle [m/s].
     * @param time The time at this state [s].
     * @param frame TODO: uhh...idk
     */
    class State{
    private:
        double x;
        double y;
        double yaw;
        double speed;
        double time;
        int frame; /** TODO: not sure about the type of 'frame'*/
    public:
        /**
         * @brief Construct a new State object with all values set to their default values.
         */
        State():x(0), y(0), yaw(0), speed(0), time(-1), frame(-1){};
        
        /**
         * @brief Construct a new State object.
         * 
         * @param x The x location [m].
         * @param y The y location [m].
         * @param yaw The yaw of the vehicle [rad].
         * @param speed The speed of the vehicle [m/s].
         * @param time The time that this State is refering to [s].
         * @param frame TODO: uhh...idk
         */
        State(double x, double y, double yaw, double speed, double time, int frame):
             x(x), y(y), yaw(yaw), speed(speed), time(time), frame(frame){};
    };


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