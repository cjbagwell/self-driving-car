/**
 * @file Location.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-06-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef LOCATION_H
#define LOCATION_H

class Location{
public:
    double x;
    double y;
    double yaw;
    // TODO: probs make the members private
    Location():x(0.0), y(0.0), yaw(0.0) {};

    /**
     * @brief Construct a new Location object
     * TODO: more stuff here
     * 
     * @param x 
     * @param y 
     * @param yaw 
     */
    Location(double x, 
             double y, 
             double yaw)
             :
             x(x),
             y(y),
             yaw(yaw)
             {};
};

#endif // LOCATION_H