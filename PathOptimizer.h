/**
 * Path Optimizer for the Local Planner
 * Author:  C. Jordan Bagwell
 * Date:    5/6/2021
 * Description: TODO: some stuff here
 */

#include<vector>
#include<iostream>
#include<ostream>

#include "Controller2D.h"
#include "LocalPlanner.h"

using namespace std;
using namespace ctr;
using namespace lpnr;

#ifndef PATH_OPTIMIZER_H
#define PATH_OPTIMIZER_H
namespace pOp{
    
    
    class PathOptimizer{
    private:
        Location lf;
    public:
        /**
         * Path Optimizer for finding Waypoint locations for Local Planner
         */
        PathOptimizer(); //TODO: don't know if I need a default constructor...
        
        /**
         * Finds set of parameters for a cubic spiral to the goal Location.
         * This function makes the following assumptions:
         * 1. The path is in the vehicle frame (ie x = 0, y=0, yaw=0)
         * 2. The curvature of the path at the start and end points is 
         *    equal to 0
         * @param locationFinal the final location for the path.
         * @returns vector of locations along the path (length n) where v[0]
         *          is at the vehicle origin and v[n-1] is the final location 
         *          of the path
         */
        vector<Location> optimizeSpiral(Location locationFinal);
    };
}

#endif