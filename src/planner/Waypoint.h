/**
 * @file Waypoint.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-06-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef WAYPOINT_H
#define WAYPOINT_H

/**
 * @brief Waypoints for a 2D Controller to track to.
 */
class Waypoint{
private:
    /** @brief The x location of the waypoint in the Navigation Frame.*/
    double x;
    /** @brief The y location of the waypoint in the Navigation Frame.*/
    double y;
    /** @brief The velocity to track at this waypoint in the Navigation Frame. TODO: in the Navigation Frame? is this necesary? is it innacurate?*/
    double v;
public:
    Waypoint():x(0), y(0), v(0) {};
    Waypoint(double x, double y, double v):x(x), y(y), v(v){};
    double getX(){return this->x;}
    double getY(){return this->y;}
    double getV(){return this->v;}
};

#endif