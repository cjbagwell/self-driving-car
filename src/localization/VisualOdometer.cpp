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

State VisualOdometer::runStep(cv::Mat currImg){
    // Extract Features and Descriptors
    vector<KeyPoint> kps1, kps2;
    cv::Mat des1, des2;
    detector->detectAndCompute(prevImg, noArray(), kps1, des1);
    detector->detectAndCompute(currImg, noArray(), kps2, des2);
    
    // Find Matching Features
    vector<DMatch> matches;
    matcher->match(des1, des2, matches);
    
    // Filter Matches
    const int thresh = 30;
    vector<DMatch> goodMatches;
    vector<Point2f> kps1m, kps2m;
    for(auto match : matches){
        if(match.distance < thresh){
            goodMatches.push_back(match);
            kps1m.push_back(kps1[match.queryIdx].pt);
            kps2m.push_back(kps2[match.trainIdx].pt);
        }
    }
    

    // Show Matches (for debugging)
    // cv::Mat outImg;
    // drawMatches(prevImg, kps1, currImg, kps2, goodMatches,outImg);
    // imshow("Matched Features", outImg);
    // waitKey(0); // Wait for a keystroke in the window

    // Get Essential Matrix
    cv::Mat E, R, t, mask;
    cout << "in runStep k:\n" << this->k << endl;
    cout << "kps1m: shape " << kps1m.size() << endl;
    cout << "kps2m: shape " << kps2m.size() << endl;

    E = cv::findEssentialMat(kps1m, kps2m, this->k, RANSAC, 0.899, 1.0, mask);
    cout << "E: \n" << E << endl;
    recoverPose(E, kps1m, kps2m, this->k, R, t, mask);
    cout << "After recoverPose" << endl;
    cout << "R:\n" << R << endl;
    cout << "t:\n" << t << endl;
    
    arma::Mat<double> trans(4,4, arma::fill::eye);
    cout << "trans\n" << trans << endl;
    arma::Mat<double> temp1(reinterpret_cast<double*>(R.data), R.rows, R.cols);
    arma::Mat<double> temp2(reinterpret_cast<double*>(t.data), t.rows, t.cols);
    trans.submat(0,0,2,2) = temp1;
    trans.submat(0, 3, 2, 3) = temp2;
    cout << "R" << "\n" << temp1 << endl;
    cout << "t" << temp2 << endl;
    cout << "Rt" << trans << endl;
    arma::Col<double> p1(4,1, arma::fill::ones);
    p1.subvec(0, 2) = prevState.pos;

    // cout << p1 << endl;
    arma::Col<double> p2;
    p2 = trans * p1;
    cout << "p1:\n" << p1 << endl;
    cout << "p2:\n" << p2 << endl;
    this->prevState.pos = p2.subvec(0,2);

    return this->prevState;
}

int main()
{
    std::string image_1_path = "/home/jordan/Projects/self-driving-car/src/localization/visualOdometry/test_image_1.png";
    std::string image_2_path = "/home/jordan/Projects/self-driving-car/src/localization/visualOdometry/test_image_2.png";
    cv::Mat img1 = imread(image_1_path, IMREAD_COLOR);
    cv::Mat img2 = imread(image_2_path, IMREAD_COLOR);
        
    
    cv::Mat k({640, 0, 640, 0, 480, 480, 0, 0, 1});
    cout << k.dims << endl;
    cout << "in main k : \n" << k << endl;

    k = k.reshape(1, {3,3});
    cout << k.dims << endl;
    cout << "in main k : \n" << k << endl;
    State s1 = State();
    VisualOdometer vo = VisualOdometer(k, img1, s1);
    State s2 = vo.runStep(img2);

    cout << "s1: \n" << s1 << endl;
    cout << "s2: \n" << s2 << endl;


    // if(k == 's')
    // {
    //     imwrite("starry_night.png", img);
    // }
    return 0;
}

// cmake -D BUILD_TIFF=ON -D WITH_CUDA=ON -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=ON -D BUILD_PERF_TESTS=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/usr/local/src/opencv_contrib/modules /usr/local/src/opencv/
