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

// std lib includes
#include<iostream>
#include<ostream>
#include<math.h>
#include<algorithm>
#include<vector>

// project includes
#include<armadillo>

#ifndef ROTATIONS_H
#define ROTATIONS_H

using namespace arma;
using namespace std;

const double PI = 3.14159265359;
const double QUAT_EQUALS_TOLERANCE = 0.000001;

/**
 * @brief Wraps the input angle to the range -PI and PI
 * 
 * @param angle angle [rad] to be wrapped between -PI and PI
 * @return double wrapped angle
 */
double angleNormalise(double& angle){
    while(true){
        if(angle <= -PI) angle += 2 * PI;
        else if(angle > PI) angle -= 2 * PI;
        else return angle;
    }   
}

/**
 * @brief TODO: some stuff here
 * 
 * @param angles 
 * @return Col<double> 
 */
Col<double> angleNormalise(Col<double> angles){
    std::vector<double> tmp;
    for(auto angle: angles){
        tmp.push_back(angleNormalise(angle));
    }
    return Col<double>(tmp);
}

/**
 * @brief TODO: some stuff here
 * 
 * @param v 
 * @return arma::Mat<double> 
 */
Mat<double> skewSemetric(Col<double> v){
    return Mat<double>({{0, -v[2], v[1]}, 
                        {v[2], 0, -v[0]}, 
                        {-v[1], v[0], 0}});
}


class Quaternion{
private:
    double w, x, y, z;
public:
    friend ostream& operator<<(ostream& out, const Quaternion& q);
    bool operator==(const Quaternion& q2){
        if((w - q2.w) < QUAT_EQUALS_TOLERANCE && 
           (x - q2.x) < QUAT_EQUALS_TOLERANCE && 
           (y - q2.y) < QUAT_EQUALS_TOLERANCE &&
           (z - q2.z) < QUAT_EQUALS_TOLERANCE){
            return true;
        }
        return false;
    }

    Quaternion(double w=1, double x=0, double y=0, double z=0):w(w), x(x), y(y), z(z){};
    Quaternion(Row<double> quaternionArray){
        this->w = quaternionArray[0];
        this->x = quaternionArray[1];
        this->y = quaternionArray[2];
        this->z = quaternionArray[3];
    }
    
    Quaternion(const Col<double>& angles, const bool& isAxisAngles){
        if(isAxisAngles){
            double norm = arma::norm(angles);
            this->w = cos(norm/2);
            if(norm < exp10(-50)){
                this->x = this->y = this->z = 0;
            }
            else{
                Col<double> imag = angles / norm * sin(norm/2);
                this->x = imag[0];
                this->y = imag[1];
                this->z = imag[2];
            }
        }
        else{ // is Euler angles
            double pitch = angles[1];
            double yaw = angles[2];
            double roll = angles[0];

            double cy = cos(yaw / 2);
            double sy = sin(yaw / 2);
            double cr = cos(roll / 2);
            double sr = sin(roll / 2);
            double cp = cos(pitch / 2);
            double sp = sin(pitch / 2);

            // Fixed frame
            // this->w = cr * cp * cy + sr * sp * sy;
            // this->x = sr * cp * cy - cr * cp * sy;
            // this->y = cr * sp * cy + sr * cp * sy;
            // this->z = cr * cp * sy - sr * sp * cy;

            // Rotating frame
            this->w = cr * cp * cy - sr * sp * sy;
            this->x = cr * sp * sy + sr * cp * cy;
            this->y = cr * sp * cy - sr * cp * sy;
            this->z = cr * cp * sy + sr * sp * cy;
        }
    }

    Quaternion operator*(const Quaternion& q){
        // should be quat_mult_left?
        Col<double> v = {x, y, z};
        Mat<double> sumTerm = arma::zeros(4,4);
        sumTerm.submat(0,1,0,3) = -v.as_row();
        sumTerm.submat(1,0,3,0) = v;
        sumTerm.submat(1,1,3,3) = skewSemetric(v);
        Mat<double> sigma = this->w * eye(4,4) + sumTerm;
        Col<double> quatArr = sigma * Col<double>({q.w, q.x, q.y, q.z}); 
        return Quaternion(quatArr.as_row());
    }

    Col<double> toAxisAngles(){
        double t = 2 * acos(std::max(-1.0, std::min(this->w, 1.0)));
        Col<double> angles = t * Col<double>({x, y, z});
        return angleNormalise(angles);
    }
    
    Col<double> toEulerAngles(){
        double roll  = atan2(2 * (w*x + y*z), 1 - 2*(x*x + y*y));
        double pitch = asin(2 * (w*y - z*x));
        double yaw   = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
        Col<double> eAngles({roll, pitch, yaw});
        return angleNormalise(eAngles);
    }

    Mat<double> toRotMat(){
        Col<double> v({x, y, z});
        Row<double> vt = v.t();
        return (w*w - dot(vt,v)) * eye(3,3) + 2*v*vt + 2*w*skewSemetric(v);
    }

    Row<double> asArma(){
        return Row<double>({w, x, y, z});
    }

    vector<double> asVector(){
        return vector<double>({w, x, y, z});
    }
    
    void normalize(){
        double norm = arma::norm(this->asArma());
        this->w = this->w / norm;
        this->x = this->x / norm;
        this->y = this->y / norm;
        this->z = this->z / norm;
        return;
    }
    
    Quaternion getInvers(){
        double norm = arma::norm(this->asArma());
        return Quaternion(this->asArma() / norm);
    }

};

ostream& operator<<(ostream& out, const Quaternion& q){
    out << "(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
    return out;
}

Quaternion eulerToQuaternion(vector<double> angles){
    Quaternion retQuat(angles, false);
    return retQuat;
}

#endif