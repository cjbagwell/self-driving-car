/**
 * @file Controller2D.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief This file contains headers of classes that are required to control 
 * the self-driving-car.  The controller only operates in 2 dimensions (x and y)
 * and is responsible calculating the required vehicle controls to reach a 
 * desired Waypoint (location and orientation) at a desired speed.
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
        std::list<double> errorBuffer; 
    public:
        /**
         * @brief Construct a new Longitudinal PID Controller object with
         * default PID gains (kp=1.0, ki=0.2, kd=0.5)
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
         * @brief Runs a single step of the control algorithm to calculate the 
         * required acceleration for the ego vehicle
         * 
         * @param currentSpeed The current speed of the ego vehicle [m/s]
         * @param targetSpeed  The desired speed of the ego vehicle [m/s]
         * @param dt The elapsed time since the previous controller step [s]
         * @return double The required acceleration output to reach the desired speed
         */
        double runStep(double currentSpeed, double targetSpeed, double dt); 
    };

    class LateralStanleyController{
    private:
        double ks, kcte;
    public:
        /**
         * @brief Construct a new Lateral Stanley Controller object with default
         * velocity softening and Crosstrack-Error gains (ks=0.1, kcte=3.0)
         */
        LateralStanleyController():ks(0.1), kcte(3.0) {};
        
        /**
         * @brief Construct a new Lateral Stanley Controller object
         * 
         * @param ks The velocity softening term.  This value allows the controller
         * to perform better when the velocity measurements are noisy, esspesially
         * when velocity is close to zero.
         * @param kcte The Crosstrack-Error gain.  Higher values of kcte will result 
         * in the controller peanalizing motion away from the desired trajectory more 
         * strongly.
         */
        LateralStanleyController(double ks, 
                                 double kcte)
                                 :
                                 ks(ks),
                                 kcte(kcte)
                                 {};
        
        /**
         * @brief Runs a single step of the control algorithm to calculate the 
         * required steering angle for the ego vehicle
         * @param currState The current state of the ego vehicle.  The state must be 
         * of the center of the front axle.
         * @param currWaypoint The Current Waypoint for the vehicle to track to.
         * @return double The required steering commands to reach the target waypoint
         * [0,1]
         */
        double runStep(State currState, Waypoint currWaypoint);
    };
    
    class Controller2D{
    private:
        std::vector<double> velocityErrorBuffer; 
        LateralStanleyController latController;
        LongitudinalPIDController lonController;

    public:
        /**
         * @brief Construct a new Controller 2 D object
         *  TODO: update docs
         * @param iniCommands 
         */
        Controller2D(double kp=0.7, 
                     double ki=0.2, 
                     double kd=0.5, 
                     double ks=0.1, 
                     double kcte=1.0)
                     :
                     lonController(kp, ki, kd),
                     latController(ks, kcte){};

        virtual ~Controller2D();

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

