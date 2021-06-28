/**
 * @file BehaviouralPlanner.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-05-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// std lib includes
#include<iostream>
#include<ostream>
#include<vector>
#include<algorithm>

// project lib includes
#include "../localization/State.h"
#include "Waypoint.h"

#ifndef BEHAVIOURAL_PLANNER_H
#define BEHAVIOURAL_PLANNER_H

using namespace std;

enum class Behaviour {FOLLOW_LANE, DECELERATE_TO_STOP, 
                      STAY_STOPPED, FOLLOW_LEADER};


class BehavioralPlanner{
private:
    double lookAhead, leadVehicleLookAhead;
    bool stopsignFences; //TODO: change this type
    int stopCount;
    Behaviour currBPState;

public:
    /**
     * Constructor for the BehaviouralPlanner
     * @param lookAhead - The look ahead distance for the planner. The planner will
     *              only consider paths that are this much further than the current
     *              egoState.
     * @param stopsignFences - TODO: add this
     * @param leadVehicleLookAhead - the maximum distance to check for lead vehicles.  If 
     *              nearest lead vehicle is further than this value, then
     *              the BehaviouralState will not become 'FOLLOW_LEADER'
     * @param egoState - the starting state of the ego vehicle.
     */
    BehavioralPlanner(double lookAhead, 
                      bool stopsignFences, 
                      double leadVehicleLookAhead,
                      State egoState)
                      :
                      lookAhead(lookAhead),
                      stopsignFences(stopsignFences),
                      leadVehicleLookAhead(leadVehicleLookAhead){
        currBPState = Behaviour::FOLLOW_LANE;
        stopCount = 0;
    }

    /**
     * Sets the lookAhead distance of the BehaviouralPlanner
     * @param lookAhead the new look ahead distance [m]
     * @returns void
    */
    void setLookAhead(double lookAhead){this->lookAhead = lookAhead;}

    /**
     * @param waypoints current waypoints to track (global frame)
     * @param egoState current state of the Ego Vehicle
     * @param closestLen distance from the ego state to the closest waypoint [m]
     * @param closestIndex index of the closest waypoint
     */
    int getGoalIndex(vector<Waypoint> waypoints, State egoState, double closestLen, double closestIndex);
    
    /**
     * Checks if there is a stop line that intersects the path of waypoints prior to 
     * the goal index.  If there is an obstruction, then the goal index will be updated to 
     * coincide with the stop line.
     * TODO: maybe add a 'sets' note for changes the vehicle's behavioural state
     * @param waypoitns current waypoints to track (global frame)
     * @param closestIndex the index of the closest waypoint to the egoVehicle
     * @param goalIndex the index of the current goalIndex.
     * @returns true if there is a stop line obstructing the path (goalIndex updated)
     */
    bool checkForStopSigns(const vector<Waypoint>& waypoints, const int& closestIndex, int& goalIndex);

    /**
     * Checks for a lead vehicle that is within the threshold distance to change the 
     * BehaviouralState of the ego vehicle.
     * TODO: maybe add a 'sets' note for changes the vehicle's behavioural state to FOLLOW_LEADER
     * @param egoState the current state of the egoVehicle
     * @param leadCarState the current state of the leadVehicle
     * @returns true if the BehaviouralState was changed to FOLLOW_LEADER
     */
    bool checkForLeadVehicle(State egoState, State leadCarState);

    /**
     * Gets the index of the closest Waypoint in waypoints.
     * @param waypoints current waypoints to track
     * @param egoState current state of the ego vehicle
     * @returns pair of 'distance to' and 'index of' the closest Waypoint 
     */
    pair<double, int> getClosestIndex(const vector<Waypoint>& waypoints, const State& egoState);
};

#endif