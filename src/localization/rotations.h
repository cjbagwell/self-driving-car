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

#ifndef ROTATIONS_H
#define ROTATIONS_H

// std lib includes
#include<iostream>
#include<ostream>
#include<math.h>
#include<algorithm>
#include<vector>

// project includes
#include<armadillo>

// =============================================================================
// Global Variables and Functions
// =============================================================================
const double PI = 3.14159265359;
const double QUAT_EQUALS_TOLERANCE = 0.000001;

/**
 * @brief Wraps the input angle to the range -PI and PI
 * 
 * @param angle angle [rad] to be wrapped between -PI and PI
 * @return double wrapped angle
 */
inline double angleNormalise(double& angle){
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
 * @return arma::Col<double> 
 */
inline arma::Col<double> angleNormalise(arma::Col<double> angles){
    std::vector<double> tmp;
    for(auto angle: angles){
        tmp.push_back(angleNormalise(angle));
    }
    return arma::Col<double>(tmp);
}

/**
 * @brief TODO: some stuff here
 * 
 * @param v 
 * @return arma::Mat<double> 
 */
inline arma::Mat<double> skewSemetric(arma::Col<double> v){
    return arma::Mat<double>({{0, -v[2], v[1]}, 
                        {v[2], 0, -v[0]}, 
                        {-v[1], v[0], 0}});
}

// =============================================================================
// Quaternions
// =============================================================================

class Quaternion{
private:
    double w, x, y, z;
public:
    friend std::ostream& operator<<(std::ostream& out, const Quaternion& q);
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
    Quaternion(arma::Row<double> quaternionArray){
        this->w = quaternionArray[0];
        this->x = quaternionArray[1];
        this->y = quaternionArray[2];
        this->z = quaternionArray[3];
    }
    
    Quaternion(const arma::Col<double>& angles, const bool& isAxisAngles){
        if(isAxisAngles){
            double norm = arma::norm(angles);
            this->w = cos(norm/2);
            if(norm < exp10(-50)){
                this->x = this->y = this->z = 0;
            }
            else{
                arma::Col<double> imag = angles / norm * sin(norm/2);
                this->x = imag[0];
                this->y = imag[1];
                this->z = imag[2];
            }
        }
        else{ // is Euler angles
            auto nAngles = angleNormalise(angles);
            double roll = nAngles[0];
            double pitch = nAngles[1];
            double yaw = nAngles[2];

            double cy = cos(yaw / 2);
            double sy = sin(yaw / 2);
            double cr = cos(roll / 2);
            double sr = sin(roll / 2);
            double cp = cos(pitch / 2);
            double sp = sin(pitch / 2);

            // // Fixed frame
            // this->w = cr * cp * cy + sr * sp * sy;
            // this->x = sr * cp * cy + cr * cp * sy;
            // this->y = cr * sp * cy - sr * cp * sy;
            // this->z = cr * cp * sy - sr * sp * cy;
            // this->normalize();
            // Rotating frame
            this->w = cr * cp * cy - sr * sp * sy;
            this->x = cr * sp * sy + sr * cp * cy;
            this->y = cr * sp * cy - sr * cp * sy;
            this->z = cr * cp * sy + sr * sp * cy;
        }
    }

    Quaternion operator*(const Quaternion& q) const {
        // should be quat_mult_left?
        arma::Col<double> v = {x, y, z};
        arma::Mat<double> sumTerm = arma::zeros(4,4);
        sumTerm.submat(0,1,0,3) = -v.as_row();
        sumTerm.submat(1,0,3,0) = v;
        sumTerm.submat(1,1,3,3) = skewSemetric(v);
        arma::Mat<double> sigma = this->w * arma::eye(4,4) + sumTerm;
        arma::Col<double> quatArr = sigma * arma::Col<double>({q.w, q.x, q.y, q.z}); 
        Quaternion retQuat(quatArr.as_row());
        // retQuat.normalize();
        return retQuat;
    }

    arma::Col<double> toAxisAngles(){
        double t = 2 * std::acos(std::max(-1.0, std::min(this->w, 1.0)));
        arma::Col<double> angles = t * arma::Col<double>({x, y, z});
        return angleNormalise(angles);
    }
    
    arma::Col<double> toEulerAngles(){
        double roll  = std::atan2(2 * (w*x + y*z), 1 - 2*(x*x + y*y));
        double pitch = std::asin(2 * (w*y - z*x));
        double yaw   = std::atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
        arma::Col<double> eAngles({roll, pitch, yaw});
        return angleNormalise(eAngles);
    }

    arma::Mat<double> toRotMat(){
        arma::Col<double> v({x, y, z});
        arma::Row<double> vt = v.t();
        return (w*w - arma::dot(vt,v)) * arma::eye(3,3) + 2*v*vt + 2*w*skewSemetric(v);
    }

    arma::Row<double> asArma(){
        return arma::Row<double>({w, x, y, z});
    }

    std::vector<double> asVector(){
        return std::vector<double>({w, x, y, z});
    }
    
    Quaternion normalize(){
        double norm = arma::norm(this->asArma());
        this->w = this->w / norm;
        this->x = this->x / norm;
        this->y = this->y / norm;
        this->z = this->z / norm;
        return *this;
    }
    
    Quaternion getInverse(){
        double norm = arma::norm(this->asArma());
        return Quaternion(this->asArma() / norm);
    }

};

inline std::ostream& operator<<(std::ostream& out, const Quaternion& q){
    out << "(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
    return out;
}

inline Quaternion eulerToQuaternion(std::vector<double> angles){
    Quaternion retQuat(angles, false);
    return retQuat;
}

inline std::vector<double> quaternionToEuler(Quaternion q){
    arma::Col<double> out = q.toEulerAngles();
    std::vector<double> ret = {out[0], out[1], out[2]};
    return ret;
}

inline std::vector<double> quaternionToMat(Quaternion q){
    arma::Mat<double> r = q.toRotMat();
    std::vector<double> rOut = std::vector<double>(9);
    for(int i=0; i < 3; i++){
        for(int j=0; j<3; j++){
            rOut[j+i*3] = r.at(i, j);
        }
    }
    return rOut;
}

#endif // ROTATIONS_H