/**
 * @file VisualOdometer.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief 
 * @TODO: some stuff here
 * @version 0.1
 * @date 2021-06-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef VISUAL_ODOMETER_H
#define VISUAL_ODOMETER_H

// std lib includes
#include<vector>
#include<ostream>
#include<iostream>

// opencv includes
#include<opencv2/core.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/features2d.hpp>

// project includes
#include "State.h"

class VisualOdometer{
private:
/** TODO: probs make these pointers and make params const */
    cv::Mat prevImg;
    cv::Mat k;
    State prevState;
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorMatcher> matcher; //BFMatcher or FlannBasedMatcher

    /** 
     * TODO: need to add camera calibration matrix k. not sure about type yet.
     * TODO: maybe keep the previous features and descriptors as well.  uses more memory but no redundant calcs
     */
    
public:

    /**
     * @brief Construct a new Visual Odometer object
     * TODO: update constructor for cv::objects
     * 
     * @param initImg The image taken at the initial state
     * @param initState The initial State of the vehicle
     */
    VisualOdometer(
                   cv::Mat calibMat,
                   cv::Mat initImg,
                   State initState)
                   :
                   k(calibMat),
                   prevImg(initImg),
                   prevState(initState),
                   detector(cv::ORB::create(500, 1.2, 9, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20)),
                   matcher(cv::BFMatcher::create(cv::NORM_HAMMING, true))
                   {};

    void setPrevState(State prevState){this->prevState = prevState;}
    
    /**
     * @brief Run a step of localization using visual odometry.  This function will compare the previous
     * image given to this object to currImg to estimate the motion of the vehicle between time
     * steps.  This objects previous image is then set to currImg.  Note that when used with a 
     * kalman filter, the corrected state should be set for this object using setPrevState.
     * 
     * @param currImg The most recent image taken.
     * @return State - The estimated state of the vehicle at the time currImg was taken (in the Navigation Frame)
     */
    State runStep(cv::Mat currImg);
};


#endif