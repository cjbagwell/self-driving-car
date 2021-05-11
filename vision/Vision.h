/**
 * Vision System for Self Driving Car
 * Author:  C. Jordan Bagwell
 * Date:    5/11/2021
 * Description: TODO: some stuff here
 */

#include<iostream>
#include<ostream>
#include<vector>
#include<list>
#include<algorithm>
#include<armadillo>
// TODO: add opencv (maybe opencv2)
#include "../controller/Controller2D.h"
#include "../localization/Localization.h"

#ifndef VISION_H
#define VISION_H

namespace vis{
    using namespace std;
    using namespace ctr;
    using namespace arma;

    class VisualOdometer{
    private:
        
        arma::Mat<double> *k; 
    public:
        /**
         * Constructor for Visual Odometer 
         * @param k the camera calibration matrix for the camera used in odometry
         * @param matchThreshold the maximum allowable relative distance between matched features
         *  across images.
         */
        VisualOdometer(arma::Mat<double> k, double matchThreshold);

        /**
         * Finds the current state of the vehicle based on motion estimated through feature matching.
         * @param previousImage the image produced while the vehicle was at it's previous state.
         * @param currentImage the image produced at the current time step.  This image will correspond with the 
         *  returned state.
         * @param previousState the state of the vehicle at the time previousImage was taken.
         * @returns the current state of the vehicle.  This will be the state of the vehicle at the moment currentImage was taken.
         * TODO: need to updated the types of previousImage and currentImage to be opencv images once the lib is up and working
         */
        State getCurrentState(arma::Mat<int> previousImage, arma::Mat<int> currentImage, const State &previousState);
    };
}


#endif