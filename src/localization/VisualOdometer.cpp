#include<iostream>
#include<ostream>
#include<vector>

#include "VisualOdometer.h"
#include "State.h"

#include<opencv2/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;

State VisualOdometer::runStep(Mat currImg){
    // Extract Features and Descriptors
    vector<KeyPoint> kps1, kps2;
    Mat des1, des2;
    detector->detectAndCompute(prevImg, noArray(), kps1, des1);
    detector->detectAndCompute(currImg, noArray(), kps2, des2);
    
    // Find Matching Features
    vector<DMatch> matches;
    matcher->match(des1, des2, matches);
    
    // Filter Matches
    const int thresh = 30;
    vector<DMatch> goodMatches;
    for(auto match : matches){
        if(match.distance < thresh){
            goodMatches.push_back(match);
        }
    }

    // Show Matches (for debugging)
    Mat outImg;
    drawMatches(prevImg, kps1, currImg, kps2, goodMatches,outImg);
    imshow("Matched Features", outImg);
    int k = waitKey(0); // Wait for a keystroke in the window

    

    return State();
}

int main()
{
    std::string image_1_path = "/home/jordan/Projects/self-driving-car/src/localization/visualOdometry/test_image_1.png";
    std::string image_2_path = "/home/jordan/Projects/self-driving-car/src/localization/visualOdometry/test_image_2.png";
    cv::Mat img1 = imread(image_1_path, IMREAD_COLOR);
    cv::Mat img2 = imread(image_2_path, IMREAD_COLOR);
    
    State s1 = State();
    VisualOdometer vo = VisualOdometer(img1, s1);
    State s2 = vo.runStep(img2);


    // if(k == 's')
    // {
    //     imwrite("starry_night.png", img);
    // }
    return 0;
}

// cmake -D BUILD_TIFF=ON -D WITH_CUDA=ON -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=ON -D BUILD_PERF_TESTS=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/usr/local/src/opencv_contrib/modules /usr/local/src/opencv/
