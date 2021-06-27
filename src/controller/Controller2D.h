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


#ifndef CONTROLLER_2D_H
#define CONTROLLER_2D_H

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


namespace controller{

    class LongitudinalPIDController{
    private:
        const double kp, ki, kd;
        std::list<double> errorBuffer; /** TODO: probably change the type of erBuf */
    public:
        /**
         * @brief Construct a new Longitudinal P I D Controller object
         * TODO: update docs
         * 
         */
        LongitudinalPIDController():kp(1.0), ki(0.2), kd(0.5) {};
        
        /**
         * @brief Construct a new Longitudinal P I D Controller object
         * TODO: update docs
         * 
         * @param kp 
         * @param ki 
         * @param kd 
         */
        LongitudinalPIDController(double kp, 
                               double ki, 
                               double kd)
                               :
                               kp(kp),
                               ki(ki),
                               kd(kd)
                               {};
        
        /**
         * @brief 
         * TODO: update docs
         * 
         * @param currentSpeed 
         * @param targetSpeed 
         * @param dt 
         * @return double 
         */
        double runStep(double currentSpeed, double targetSpeed, double dt); 
    };

    class LateralStanleyController{
    private:
        double ks, kcte;
    public:
        /**
         * @brief Construct a new Lateral Stanley Controller object
         * TODO: update docs
         * 
         */
        LateralStanleyController():ks(0.1), kcte(1.0) {};
        
        /**
         * @brief Construct a new Lateral Stanley Controller object
         * TODO: update docs
         * 
         * @param ks 
         * @param kcte 
         */
        LateralStanleyController(double ks, 
                                 double kcte)
                                 :
                                 ks(ks),
                                 kcte(kcte)
                                 {};
        
        /**
         * @brief 
         * TODO: update docs
         * 
         * @param currState NOTE: the position of state is assumed to be at front axle
         * @param prevWaypoint 
         * @param currWaypoint 
         * @return double 
         */
        double runStep(State currState, Waypoint currWaypoint);
    };
    
    class Controller2D{
    private:
        std::vector<Waypoint> waypoints;
        std::vector<double> velocityErrorBuffer; 
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
        Controller2D(std::vector<Waypoint> ws, Commands iniCommands):waypoints(ws), prevCommands(iniCommands){};
        
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
        void updateWaypoints(const std::vector<Waypoint> &newWaypoints){this->waypoints = newWaypoints;}
        
        /**
         * @brief 
         * TODO: update docs
         * 
         * @param currState 
         * @param prevWaypoint 
         * @param currWaypoint 
         * @return Commands 
         */
        Commands runStep(State currState, Waypoint currWaypoint, double dt);
    };
    
    

}
#endif

