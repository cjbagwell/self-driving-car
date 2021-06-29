/**
 * @file ImuMeasurement.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief 
 * TODO: some stuff here
 * @version 0.1
 * @date 2021-06-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef IMU_MEASUREMENT_H
#define IMU_MEASUREMENT_H

// std lib includes
#include<iostream>
#include<ostream>
#include<vector>

// project includes
#include<armadillo>

class ImuMeasurement{
private:
public:
    arma::Col<double> accelerometer, gyro;
    double compas, time;
    int frame;

    friend std::ostream& operator<<(std::ostream& out, const ImuMeasurement& m);

    double getTime(){return this->time;}

    ImuMeasurement():accelerometer(3), compas(0), gyro(3), time(-1), frame(-1){};
    
    /**
     * @brief Construct a new Imu Measurement object
     * 
     * @param a accelerometer measurement vector 
     * @param c compas measurement
     * @param g gyroscope measurement vector
     * @param t time the measurement occured at
     * @param f frame the measurement occured during
     */
    ImuMeasurement(arma::Col<double> a, 
                   double c, 
                   arma::Col<double> g,
                   double t,
                   int f)
                   :
                   accelerometer(a),
                   compas(c),
                   gyro(g),
                   time(t),
                   frame(f)
                   {};
    
    /**
     * @brief Construct a new Imu Measurement object
     * 
     * @param a accelerometer measurement from the IMU. Must be length 3.
     * @param c compas measurement from the IMU. Must be length 3.
     * @param g gryoscope measurement from the IMU. Must be length 3.
     * @param t time the measurement occured at.
     */
    ImuMeasurement(std::vector<double> a,
                   double c,
                   std::vector<double> g,
                   double t,
                   int f)
                   :
                   accelerometer(a),
                   compas(c),
                   gyro(g),
                   time(t),
                   frame(f)
    {
        if(a.size() != 3){
            std::cout << "Accelerometer measurement must be of length 3" << std::endl;
            throw -1;
        }
        if(g.size() != 3){
            std::cout << "Gyroscope measurement must be of length 3" << std::endl;
            throw -1;
        }    
    };
};

std::ostream& operator<<(std::ostream& out, const ImuMeasurement& m){
    out << "{accel:" << m.accelerometer <<
            ", comp:" << m.compas <<
            ", gyro:" << m.gyro;
    return out;
}

#endif