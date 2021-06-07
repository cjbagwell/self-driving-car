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

#include<armadillo>
#include "rotations.h"

using namespace std;
using namespace arma;

class State{
public:
    /** TODO: probs make the members private */
    Col<double> pos;
    Col<double> vel;
    Quaternion rot;    
    
    double time;
    int frame; /** TODO: not sure about the type of 'frame'*/

    /**
     * @brief Construct a new State object with all values set to their default values.
     */
    State():pos(3), vel(3), rot(), time(-1), frame(-1){};
    
    /**
     * @brief Construct a new State object
     * 
     * @param pos initial position vector
     * @param vel initial velcotiy vector
     * @param rot initial rotation Quaternion
     * @param time time this state was calculated
     * @param frame frame that this state was calcualted
     */
    State(Col<double> pos, 
          Col<double> vel, 
          Quaternion rot, 
          double time=-1, 
          double frame=-1)
          :
          pos(pos),
          vel(vel),
          rot(rot),
          time(time),
          frame(frame)
          {};
    
    State(vector<double> pos,
          vector<double> vel,
          vector<double> rot,
          double time,
          int frame)
          :
          pos(pos),
          vel(vel),
          rot(rot),
          time(time),
          frame(frame)
          {};

    double getSpeed(){
        return arma::norm(vel);
    }
};
