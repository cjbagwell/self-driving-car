/**Self-driving Car 2D Controller
 * Author:  C. Jordan Bagwell
 * Date:    May 3rd, 2021
 */

#include<iostream>
#include<ostream>
#include<vector>
#include<tuple>
using namespace std;

/*
code design
class Controller2D
*/



struct Waypoint{
    double x;
    double y;
    double v;
} typedef Waypoint;

struct State{
    double x;
    double y;
    double yaw;
    double speed;
    double time;
    int frame; //TODO: not sure about the type of 'frame'
} typedef State;

struct Commands{
    double app;
    double bpp;
    double steerAngleRate;
} typedef Commands;

class Controller2D{
public:
    Controller2D(vector<Waypoint> ws):waypoints(ws){}
    virtual ~Controller2D();
    void updateState(State &egoState){
        this->prevState = this->currState; 
        this->currState = egoState;
    }
    void updateWaypoints(vector<Waypoint> &newWaypoints){this->waypoints = newWaypoints;}
    void updateDesiredSpeed();
    
    Commands getCommands(){return this->currCommands;};
    bool updateCommands();

private:
    State currState, prevState;
    Commands currCommands;
    vector<Waypoint> waypoints;
    double vDesired, vDesiredPrev, vErrorPrev, vErrorInt; 
};