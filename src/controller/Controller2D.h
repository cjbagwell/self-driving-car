/**
 * @file Controller2D.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here 
 * @version 0.2
 * @date 2021-05-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// std lib includes
#include<iostream>
#include<ostream>
#include<vector>
#include<list>
#include<tuple>

// project libs includes
#include "../localization/State.h"
#include "../planner/Waypoint.h"
#include "../planner/Commands.h"

#ifndef CONTROLLER_2D_H
#define CONTROLLER_2D_H

namespace controller{

    using namespace std;

    class LongitudinalPIDController{
    private:
        const double kp, ki, kd;
        list<double> errorBuffer; /** TODO: probably change the type of erBuf */
    public:
        LongitudinalPIDController():kp(1.0), ki(0.2), kd(0.5) {};
        LongitudinalPIDController(double kp, 
                               double ki, 
                               double kd)
                               :
                               kp(kp),
                               ki(ki),
                               kd(kd)
                               {};
        
        double runStep(double currentSpeed, double targetSpeed, double dt); 
    };

    class LateralStanleyController{
    private:
        double ks, kcte;
    public:
        LateralStanleyController():ks(0.1), kcte(1.0) {};
        LateralStanleyController(double ks, 
                                 double kcte)
                                 :
                                 ks(ks),
                                 kcte(kcte)
                                 {};
        double runStep(State currState, Waypoint prevWaypoint, Waypoint currWaypoint);
    };
    
    class Controller2D{
    private:
        vector<Waypoint> waypoints;
        vector<double> velocityErrorBuffer; 
        Commands prevCommands;
        LateralStanleyController latController;
        LongitudinalPIDController lonController;

    public:
        /**
         * @brief Construct a new Controller 2D object
         * TODO: update these docs
         * 
         * @param ws the initial waypoints for the controller to track to.  See
         * ctr::Waypoint for more information.
         */
        Controller2D(vector<Waypoint> ws, Commands iniCommands):waypoints(ws), prevCommands(iniCommands){};
        
        /**
         * @brief Construct a new Controller 2 D object
         *  TODO: update docs
         * @param iniCommands 
         */
        Controller2D(Commands iniCommands, 
                     double kp=0.7, 
                     double ki=0.2, 
                     double kd=0.5, 
                     double ks=0.1, 
                     double kcte=1.0)
                     :
                     prevCommands(iniCommands),
                     lonController(kp, ki, kd),
                     latController(ks, kcte){};

        virtual ~Controller2D();

        /**
         * @brief This method updates the waypoints that controller will track to.  All of the
         * waypoints that the controller is currently tracking will be replaced by newWaypoints.
         * 
         * @param newWaypoints The new waypoints that the controller will track to.  See ctr::Waypoint
         * for more information.
         */
        void updateWaypoints(const vector<Waypoint> &newWaypoints){this->waypoints = newWaypoints;}
        
        /**
         * @brief TODO: update docs
         * 
         * @param currState 
         * @param prevWaypoint 
         * @param currWaypoint 
         * @return Commands 
         */
        Commands runStep(State currState, Waypoint prevWaypoint, Waypoint currWaypoint, double dt);
    };
    
    

}
#endif

