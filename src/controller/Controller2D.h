/**Self-driving Car 2D Controller
 * Author:  C. Jordan Bagwell
 * Date:    May 3rd, 2021
 */

#include<iostream>
#include<ostream>
#include<vector>
#include<tuple>
#include<pybind11/pybind11.h>
#ifndef CONTROLLER_2D_H
#define CONTROLLER_2D_H

namespace ctr{

    using namespace std;
    namespace py = pybind11;

    /**
     * @brief Waypoints for a 2D Controller to track to.
     */
    class Waypoint{
    public:
        /** @brief The x location of the waypoint in the Navigation Frame.*/
        double x;
        /** @brief The y location of the waypoint in the Navigation Frame.*/
        double y;
        /** @brief The velocity to track at this waypoint in the Navigation Frame. TODO: in the Navigation Frame? is this necesary? is it innacurate?*/
        double v;
        Waypoint():x(0), y(0), v(0) {};
        Waypoint(double x, double y, double v):x(x), y(y), v(v){};
    };

    /**
     * @brief A State of a vehicle to be used for a self-driving car.
     * @param x The x location of the vehicle[m].
     * @param y The y location of the vehicle [m].
     * @param yaw The yaw of the vehicle [rad].
     * @param speed The speed of the vehicle [m/s].
     * @param time The time at this state [s].
     * @param frame TODO: uhh...idk
     */
    class State{
    public:
        /**
         * @brief Construct a new State object with all values set to their default values.
         */
        State():x(0), y(0), yaw(0), speed(0), time(-1), frame(-1){};
        
        /**
         * @brief Construct a new State object.
         * 
         * @param x The x location [m].
         * @param y The y location [m].
         * @param yaw The yaw of the vehicle [rad].
         * @param speed The speed of the vehicle [m/s].
         * @param time The time that this State is refering to [s].
         * @param frame TODO: uhh...idk
         */
        State(double x, double y, double yaw, double speed, double time, int frame):
             x(x), y(y), yaw(yaw), speed(speed), time(time), frame(frame){};
        double x;
        double y;
        double yaw;
        double speed;
        double time;
        int frame; /** TODO: not sure about the type of 'frame'*/
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
    struct Commands{
        double app;
        double bpp;
        double steerAngleRate;
    } typedef Commands;

    class Controller2D{
    private:
        State currState, prevState;
        Commands currCommands;
        vector<Waypoint> waypoints;
        double vDesired, vDesiredPrev, vErrorPrev, vErrorInt; 
    public:
        /**
         * @brief Construct a new Controller 2 D object
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
    

}
#endif