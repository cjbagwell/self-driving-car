#include<iostream>
#include<ostream>
#include<vector>

#include "VisualOdometer.h"
#include "matplotlibcpp.h"
#include "State.h"

#include<opencv2/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;
namespace plt = matplotlibcpp;

void visualizeMotion(const cv::Mat& im, 
                     const vector<KeyPoint>& kpsCurr, 
                     const vector<KeyPoint>& kpsPrev, 
                     const vector<DMatch>& matches, 
                     cv::Mat& outIm){

    im.copyTo(outIm);
    for(auto m : matches){
        cv::circle(outIm, kpsPrev[m.trainIdx].pt, 5, Scalar(0, 0,255), 1);
        cv::circle(outIm, kpsCurr[m.queryIdx].pt, 5, Scalar(0, 255, 0), 1);
        
        cv::arrowedLine(outIm, kpsPrev[m.trainIdx].pt, kpsCurr[m.queryIdx].pt, Scalar(0,0,0));
        // cv::imshow("Matched Features", outIm);
        // waitKey();
    }                      
    return;
}

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
    const float thresh = 0.7;
    const int distThresh = 30;
    vector<DMatch> goodMatches;
    vector<Point2f> kps1m, kps2m;
    for(auto match : matches){        
        if(match.distance < distThresh){
            goodMatches.push_back(match);
            kps1m.push_back(kps1[match.trainIdx].pt);
            kps2m.push_back(kps2[match.queryIdx].pt);
            // cout << "Adding a match!" << endl;
        }
    }
    

    // Show Matches (for debugging)
    cv::Mat outImg;
    visualizeMotion(currImg, kps1, kps2, goodMatches, outImg);
    imshow("Matched Features", outImg);
    waitKey(0); // Wait for a keystroke in the window

    // Recover Pose
    cv::Mat E, R, t, mask;
    E = cv::findEssentialMat(kps1m, kps2m, this->k, RANSAC, 0.899, 1.0, mask);
    cout << "E: \n" << E << endl;
    recoverPose(E, kps1m, kps2m, this->k, R, t, mask);
    cout << "R:\n" << R << endl;
    
    arma::Mat<double> trans(4,4, arma::fill::eye);
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
    this->prevImg = currImg;

    return this->prevState;
}

int main()
{
    std::string image_dir_path = "/home/jordan/Projects/self-driving-car/src/localization/test/images/";
    cv::Mat k({640, 0, 640, 0, 480, 480, 0, 0, 1});
    cout << k.dims << endl;
    cout << "in main k : \n" << k << endl;

    k = k.reshape(1, {3,3});
    cout << k.dims << endl;
    cout << "in main k : \n" << k << endl;
    State s1 = State();
    VisualOdometer vo = VisualOdometer(k, img, s1);
    int maxNum = 100;
    int imNum = 1;
    vector<State> outputState(maxNum);
    vector<double> xOut(maxNum), yOut(maxNum), zOut(maxNum);
    while(true){
        std::string imName = image_dir_path + "test_image_" + to_string(imNum) + ".png";
        cout << "Image Name: " << imName << endl;
        try{
            img = imread(imName, IMREAD_COLOR);
            State s2 = vo.runStep(img);
            outputState.push_back(s2);
            xOut.push_back(s2.pos[0]);
            yOut.push_back(s2.pos[1]);
            zOut.push_back(s2.pos[2]);
            cout << "Predicted State: \n" << s2.pos << endl;
        }
        catch(int e){
            break;
        }
        if(++imNum > maxNum) break;
    }

    plt::plot(xOut, yOut, "--r");
    plt::show();
    
    // if(k == 's')
    // {
    //     imwrite("starry_night.png", img);
    // }
    return 0;
}

// cmake -D BUILD_TIFF=ON -D WITH_CUDA=ON -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=ON -D BUILD_PERF_TESTS=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/usr/local/src/opencv_contrib/modules /usr/local/src/opencv/
