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
// std lib includes
#include<iostream>
#include<ostream>
#include<vector>

// project includes
#include<armadillo>

using namespace arma;
using namespace std;

#ifndef IMU_MEASUREMENT_H
#define IMU_MEASUREMENT_H

class ImuMeasurement{
private:
public:
    Row<double> accelerometer;
    double compas, time;
    Row<double> gyro;

    friend ostream& operator<<(ostream& out, const ImuMeasurement& m);

    ImuMeasurement():accelerometer(3), compas(0), gyro(3), time(-1){};
    
    /**
     * @brief Construct a new Imu Measurement object
     * 
     * @param a accelerometer measurement vector 
     * @param c compas measurement
     * @param g gyroscope measurement vector
     * @param t time the measurement occured at
     */
    ImuMeasurement(Row<double> a, 
                   double c, 
                   Row<double> g,
                   double t)
                   :
                   accelerometer(a),
                   compas(c),
                   gyro(g),
                   time(t)
                   {};
    
    /**
     * @brief Construct a new Imu Measurement object
     * 
     * @param a accelerometer measurement from the IMU. Must be length 3.
     * @param c compas measurement from the IMU. Must be length 3.
     * @param g gryoscope measurement from the IMU. Must be length 3.
     * @param t time the measurement occured at.
     */
    ImuMeasurement(vector<double> a,
                   double c,
                   vector<double> g,
                   double t)
                   :
                   accelerometer(a),
                   compas(c),
                   gyro(g),
                   time(t)
    {
        if(a.size() != 3){
            cout << "Accelerometer measurement must be of length 3" << endl;
            throw -1;
        }
        if(g.size() != 3){
            cout << "Gyroscope measurement must be of length 3" << endl;
            throw -1;
        }    
    };

    friend ostream& operator<<(ostream& out, const ImuMeasurement& m){
        out << "{accel:" << m.accelerometer <<
               ", comp:" << m.compas <<
               ", gyro:" << m.gyro;
        return out;
    }
};

#endif