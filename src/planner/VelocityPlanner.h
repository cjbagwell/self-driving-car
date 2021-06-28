/**
 * @file VelocityPlanner.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-05-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// std lib includes
#include<vector>
#include<iostream>

// project lib includes
#include "Waypoint.h"

#ifndef VELOCITY_PLANNER_H
#define VELOCITY_PLANNER_H

using namespace std;

class VelocityPlanner{
private:
    double timeGap, aMax, vSlow, stopLineBuffer;
    vector<Waypoint> prevTrajectory;

public:
    /**
     * @brief TODO: some stuff here
     * 
     * @param timeGap The amount of time to maintain between the ego and lead Vehicle.
     *              The time gap is defined as the amount of time for the ego vehicle to 
     *              reach the state of the lead Vehicle. [s]
     * @param aMax The maximum acceleration/deceleration the ego vehicle is allowed to perform [m/s^2]
     * @param vSlow The coasting speed of the ego vehicle when approaching a stop [m/s].
     * @param stopLineBuffer the distance the ego vehicle should maintain from a stop line (i.e. stop sign)
     *              when stopped [m].
     */
    VelocityPlanner(double timeGap, double aMax, double vSlow, double stopLineBuffer);

    /**Computes the open loop speed estimate based on the previous trajectory
     * @param timeStep the time step since the last planning cycle [s]
     * @returns open loop speed estimate [m/s]
     */
    double getOpenLoopSpeed(double timeStep);
};

#endif