/**
 * @brief 
 * Author:  C. Jordan Bagwell
 * Date:    May 3rd, 2021
 * Description: TODO: some stuff here
 */

// std lib includes
#include<iostream>
#include<ostream>
#include<vector>
#include<tuple>

// project libs includes
#include "../localization/Localization.h"
#include "../planner/LocalPlanner.h"


#ifndef CONTROLLER_2D_H
#define CONTROLLER_2D_H

namespace controller{

    using namespace std;
    using namespace lcl;
    using namespace lpnr;

    class Controller2D{
    private:
        State currState, prevState;
        Commands currCommands;
        vector<Waypoint> waypoints;
        double vDesired, vDesiredPrev, vErrorPrev, vErrorInt; 
    public:
        /**
         * @brief Construct a new Controller 2D object
         * 
         * @param ws the initial waypoints for the controller to track to.  See
         * ctr::Waypoint for more information.
         */
        Controller2D(vector<Waypoint> ws):waypoints(ws){}
        
        virtual ~Controller2D();
        
        /**
         * @brief This method sets the previous state of the controller to the 
         * controller's current state and then sets the current state of the 
         * controller to egoState.
         * 
         * @param egoState The current state of the ego vehicle.  This should come 
         * from the locoalization module of the self-driving car or directly from
         * the simulator (for testing purposes).
         */
        void updateState(const State &egoState);

        /**
         * @brief This method updates the waypoints that controller will track to.  All of the
         * waypoints that the controller is currently tracking will be replaced by newWaypoints.
         * 
         * @param newWaypoints The new waypoints that the controller will track to.  See ctr::Waypoint
         * for more information.
         */
        void updateWaypoints(const vector<Waypoint> &newWaypoints){this->waypoints = newWaypoints;}
        
        /**
         * @brief updates the desired speed of the controller to the speed at the closest Waypoint.
         * TODO: this should be private or a friend or something else...should not be called in the interface.
         */
        void updateDesiredSpeed();
        
        /**
         * @brief Get the current commands for the self-driving car, based on the information previously 
         * given to the controller.  A typical implementation would be to update the waypoints using 
         * updateWaypoints(); updateState(); updateCommands(); and finally getCommands();
         * 
         * @return The current Commands for the vehicle to execute.  See ctr::Commands for more information.
         */
        Commands getCommands(){return this->currCommands;};
        
        /**
         * @brief updates the commands of the controller based on the current state of the controller and
         * the current waypoints that the controller is tracking to.  It is important for the performance of the
         * controller that this is called after 'updateState(State)' is called;  this ensures the controller will
         * calculate appropriate Commands for the current state of the ego Vehicle. 
         * 
         * @return Returns true if the commands update was successful.
         * Returns false if the commands update was not successful.  In this case, the Commands of the controller
         * will remain unchanged from the previous time step.
         */
        bool updateCommands();    
    };
    
    class LongitudinalController{
    private:
    public:
        LongitudinalController();
    };

    class LateralController{
    private:
    public:
        LateralController();
    };

}
#endif

