/**
 * @file State.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-06-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

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
    /** TODO: probs make the members private */
    double x;
    double y;
    double yaw;
    double speed;
    double time;
    int frame; /** TODO: not sure about the type of 'frame'*/

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
};
