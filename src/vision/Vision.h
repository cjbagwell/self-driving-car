/**
 * @file Vision.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-05-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// std lib includes
#include<iostream>
#include<ostream>
#include<vector>
#include<list>
#include<algorithm>
#include<armadillo>

// project lib includes
/** TODO: need to add opencv as a submodule */
// #include<opencv2/highgui.hpp>
// #include<opencv2/dnn.hpp>
// #include<opencv2/imgproc.hpp>
#include "../localization/State.h"

#ifndef VISION_H
#define VISION_H

namespace vis{
    using namespace std;
    using namespace arma;
    using namespace cv;

    class VisualOdometer{
    private:
        double matchThreshold;
        arma::Mat<double> k; 
    public:
        /**
         * Constructor for Visual Odometer 
         * @param k the camera calibration matrix for the camera used in odometry
         * @param matchThreshold the maximum allowable relative distance between matched features
         *  across images.
         */
        VisualOdometer(arma::Mat<double> k, double matchThreshold):k(k), matchThreshold(matchThreshold){}

        /**
         * Finds the current state of the vehicle based on motion estimated through feature matching.
         * @param previousImage the image produced while the vehicle was at it's previous state.
         * @param currentImage the image produced at the current time step.  This image will correspond with the 
         *  returned state.
         * @param previousState the state of the vehicle at the time previousImage was taken.
         * @returns the current state of the vehicle.  This will be the state of the vehicle at the moment currentImage was taken.
         */
        State getCurrentState(cv::Mat previousImage, cv::Mat currentImage, const State &previousState);

    };

    class ObjectDetector{
    private:
        double confidenceThreshold;
    public:
        /**
         * Object Detector for self driving car.
         * TODO: need to add more info
         */
        ObjectDetector(double confidenceThreshold);

        /**
         * finds all objects in the image
         * @param image the image to detect objects in 
         * @returns all object detections found in the image. 
         * TODO: add more info about return type (bboxes, class, confidence)
         */
        vector<cv::Rect> getBboxes(const cv::Mat &image);

        /**
         * Creates a visualization for the found BBoxes in an image.  This is used with the output of 
         * 'getBboxes' and simply draws the boxes on the image.
         * @param image the image containing the objects
         * @param bboxes the bounding boxes for each object in the image
         */
        void visualizeBboxes(const cv::Mat &image, const vector<cv::Rect> bboxes);
    };

    class SemanticSegmenter{
    private:
    public:
        /**
         * Semantic Segmentation object for Self-Driving Car
         * TODO: update params to match requirements
         */
        SemanticSegmenter();

        /**
         * Returns the semantic segmentation model output of the input image.  The classes of the 
         * segmenter are TODO: add info about classes/add enum class for classes?
         * @param image the input image to perform segmentation on
         * @returns the outpout of the semantic segmentation network
         */
        cv::Mat getSegmentation(const cv::Mat &image);

    };
}

#endif