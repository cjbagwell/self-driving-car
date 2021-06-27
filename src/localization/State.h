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

#ifndef STATE_H
#define STATE_H

// std lib includes
#include<iostream>
#include<ostream>

// project includes
#include<armadillo>
#include "rotations.h"

class State{
public:
    /** TODO: probs make the members private */
    arma::Col<double> pos;
    arma::Col<double> vel;
    Quaternion rot;    
    double time;
    int frame; /** TODO: not sure about the type of 'frame'*/

    friend std::ostream& operator<<(std::ostream& out, const State& s);

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
    State(arma::Col<double> pos, 
          arma::Col<double> vel, 
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
    
    State(std::vector<double> pos,
          std::vector<double> vel,
          std::vector<double> rot,
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

    std::vector<double> getPosition(){return {pos[0], pos[1], pos[2]};}
    std::vector<double> getVelocity(){return {vel[0], vel[1], vel[2]};}
};

inline std::ostream& operator<<(std::ostream& out, const State& s){
    out << "State {frame:" << s.frame << 
        "time:" << s.time << 
        "pos:" << s.pos << 
        " vel:" << s.vel << 
        "rot:" << s.rot << "}";
    return out;
}

#endif // STATE_H