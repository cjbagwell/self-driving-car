/**
 * @file GnssMeasurement.h
 * @author your name (cjbagwell@crimson.ua.edu)
 * @brief 
 * TODO: some stuff here
 * @version 0.1
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#ifndef GNSS_MEASUREMENT_H
#define GNSS_MEASUREMENT_H

#include<vector>
#include<armadillo>
const double RADIUS_EARTH = 6.371 * 1000000;

class GnssMeasurement{
public:
    const int frame;
    const double t, alt, lat, lon;
    
    
    GnssMeasurement(int frame, 
                    double t, 
                    double alt, 
                    double lat, 
                    double lon)
                    :
                    frame(frame),
                    t(t),
                    alt(alt),
                    lat(lat),
                    lon(lon)
                    {}

    arma::Col<double> getLocation(){
        return arma::Col<double>({
            lon*RADIUS_EARTH,
            -lat*RADIUS_EARTH,
            alt-1
        });
    }
};

std::vector<double> GnssMeasurement2Position(GnssMeasurement m){
    arma::Col<double> pos = m.getLocation();
    std::vector<double> out({pos[0], pos[1], pos[2]});
    return out;
}

#endif