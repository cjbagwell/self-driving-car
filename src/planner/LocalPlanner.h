/**Local Planner for Self Driving Car
 * Author:  C. Jordan Bagwell
 * Date:    5/5/2021
 * Description: TODO: some stuff here
 */

#include<iostream>
#include<ostream>
#include<vector>
#include<set>
#include "../controller/Controller2D.h"
#include "../localization/Localization.h"
#include "CollisionChecker.h"
#include "VelocityPlanner.h"

#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

namespace localPlanner{
    using namespace std;
    using namespace controller;
    using namespace localization;

    struct Location{
            double x;
            double y;
            double yaw;
    } typedef Location;

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

    /**
     * @brief Controls that can be executed by a self-driving car.
     * 
     * @param app The accelerator pedal position (APP) for a vehicle.  This is represented as a
     * value between 0 and 1 where 0 represents no APP and 1 represents maximum APP.
     * 
     * @param bpp The brake pedal position (BPP) for a vehicle.  This is represented as a value
     * between 0 and 1 where 0 represents no BPP and 1 represents maximum BPP.
     * 
     * @param steerAngleRate The rate at which the steering agle should of the vehicle should change.
     * This is expressed in radians per second [rad/s]
     * 
     */
    class Commands{
    private:
        double app;
        double bpp;
        double steerAngleRate;
    public:
        Commands(double app, 
                 double bpp, 
                 double steerAngleRate)
                 :
                 app(app), 
                 bpp(bpp), 
                 steerAngleRate(steerAngleRate) 
                 {};
    };


    class LocalPlanner{
    private:
        int numPaths;
        double pathOffset;
        CollisionChecker *collisionChecker;
        VelocityPlanner *velocityPlanner; 
        
    public:
        /**Constructor for the Local Planner
         * The local planner is responsible for generating collision free trajectories that are
         * are kenematically feasible. TODO: spelling??
         * Paths are generated based off of parameters given by the Behavioural Planner
         * @param   numPaths number of paths for the local planner to generate
         *              for consideration.
         * @param   pathOffset TODO: add this
         * @param   circleOffsets TODO: add this
         * @param   circleRadius The radius that each circle will have for the collision checker
         * @param   timeGap The amount of time to maintain between the ego and lead Vehicle.
         *              The time gap is defined as the amount of time for the ego vehicle to 
         *              reach the state of the lead Vehicle.  This is used in velocity profile generation.
         * @param   aMax The maximum acceleration/deceleration the ego vehicle is allowed to perform [m/s^2]
         * @param   vSlow The coasting speed of the ego vehicle when approaching a stop.  This is 
         *              used for velocity profile generation [m/s].
         * @param   stopLineBuffer the distance the ego vehicle should maintain from a stop line (i.e. stop sign)
         *              when stopped [m].
         */
        LocalPlanner(int numPaths, double pathOffset, vector<double> circleOffsets,
                    double circleRadius, double timeGap, double aMax, double vSlow,
                    double stopLineBuffer);

        /**Destructor for Local Planner*/
        virtual ~LocalPlanner();
        
        /**Returns a set of possible goal states to consider for paths.
         * @param goalIndex the index of the goalWaypoint in waypoints. ie waypoints[goalIndex] == goalWaypoint
         * @param egoState the state of the ego vehicle (global frame)
         * @param goalWaypoint the goal waypoint (global frame) from waypoints.  
         * @param waypoint current waypoints to track. 
         * @returns a set of States which can be considered as ending states for a path
         */ 
        set<State> getGoalStateSet(int goalIndex, State egoState, Waypoint goalWaypoint, vector<Waypoint> waypoints);
        
        /**Plans a set of paths to each of the goal states
         * @param goalStateSet TODO: this
         * @returns a vector of paths and a vector for the validity of each path (true for value path, false for invalid path) 
         */
        pair<vector<vector<Waypoint>>,vector<bool>> planPaths(set<State> goalStateSet);
    };
}
#endif
