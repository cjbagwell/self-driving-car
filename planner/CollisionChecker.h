/**Collision Checker
 * Author:  C. Jordan Bagwell
 * Date:    5/5/2021
 * Description: TODO: some stuff here
 */

#include<iostream>
#include<ostream>
#include<vector>
#include "Controller2D.h"

using namespace std;
using namespace ctr;

class CollisionChecker{
private:
    double circleRadius, circle;
    vector<double> circleOffsets;
public:
    /**Checks each path in 'paths' for collisions with 'obstacles'
     * param: paths - all possible paths to check
     * param: obstacles - vector of locations (x, y) of the border of all obstacles to consider
     * returns: vector of length paths.size() where the ith index corresponds to the ith path.
     *          value of true indicates that the path will result in a collision.
     */
    vector<bool> checkForCollisions(const vector<vector<Waypoint>> &paths, //TODO: need to use something other than 'Waypoint' 
                                    const vector<pair<double, double>>& obstacles);
    
    /**selects the best path from 'paths' and returns the index of the best path.
     * param: paths - all possible paths to consider.
     * param: collisionCheckVec - vector of same length as paths where 'true' represents
     *                            path contains a collision
     * param: goalState - the goal state for the ego vehicle. (Normally the final state of the centerline)
     */
    int selectBestPathIndex(const vector<vector<Waypoint>> &paths, //TODO: need to replace 'Wayponits' with different type
                            const vector<bool> &collisionCheckVec,
                            const State &goalState);
};