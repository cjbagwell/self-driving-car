/**
 * @file rotations.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief 
 * TODO: some stuff here
 * @version 0.1
 * @date 2021-06-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include<armadillo>
#include<iostream>
#include<ostream>
#include<math.h>

#ifndef ROTATIONS_H
#define ROTATIONS_H

using namespace arma;

/**
 * @brief TODO: some stuff here
 * 
 * @param a angle to normalize [rad]
 * @return double normalized angle [rad]
 */
double angleNormalize(double a);

/**
 * @brief TODO: some stuff here
 * 
 * @param v 
 * @return arma::Mat<double> 
 */
Mat<double> skewSemetric(Col<double> v);


class Quaternion{
private:
    double w, x, y, z;
public:
    Quaternion(double w=1, double x=0, double y=0, double z=0):w(w), x(x), y(y), z(z){};
    Quaternion(Row<double> quaternionArray){
        this->w = quaternionArray[0];
        this->x = quaternionArray[1];
        this->y = quaternionArray[2];
        this->z = quaternionArray[3];
    }
    
    Quaternion(const Row<double>& angles, const bool& isAxisAngles){
        if(isAxisAngles){
            double norm = arma::norm(angles);
            this->w = cos(norm);
            if(norm < exp10(-50)){
                this->x = this->y = this->z = 0;
            }
            else{
                Row<double> imag = angles / norm * sin(norm/2);
                this->x = imag[0];
                this->y = imag[1];
                this->z = imag[2];
            }
        }
        else{ // is Euler angles
            double roll = angles[0];
            double pitch = angles[1];
            double yaw = angles[2];

            double cy = cos(yaw / 2);
            double sy = sin(yaw / 2);
            double cr = cos(roll / 2);
            double sr = sin(roll / 2);
            double cp = cos(pitch / 2);
            double sp = sin(pitch / 2);

            // Fixed frame
            this->w = cr * cp * cy + sr * sp * sy;
            this->x = sr * cp * cy - cr * cp * sy;
            this->y = cr * sp * cy + sr * cp * sy;
            this->z = cr * cp * sy - sr * sp * cy;

            // Rotating frame
            // this->w = cr * cp * cy - sr * sp * sy;
            // this->x = cr * sp * sy + sr * cp * cy;
            // this->y = cr * sp * cy - sr * cp * sy;
            // this->z = cr * cp * sy + sr * sp * cy;
        }
    }

    Quaternion operator*(const Quaternion& q){
        // should be quat_mult_left?
        Col<double> v = {x, y, z};
        Mat<double> sumTerm = arma::zeros(4,4);
        sumTerm.submat(0,1,0,3) = -v.col(0);
        sumTerm.submat(1,0,3,0) = v.col(0);
        sumTerm.submat(1,1,3,3) = skewSemetric(v);
        Mat<double> sigma = this->w * eye(4,4) + sumTerm;
        Row<double> quatArr = sigma * Row<double>({q.w, q.x, q.y, q.z});
        return Quaternion(quatArr);
    }

    Row<double> toAxisAngles(); /** TODO: implement this method*/
    Row<double> toEulerAngles(); /** TODO: implement this method*/
    Mat<double> toRotMat(); /** TODO: implement this method*/
    Row<double> asVector(){
        return Row<double>({w, x, y, z});
    }
    void normalize(); /** TODO: implement this method*/
    
};


#endif