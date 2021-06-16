#include <iostream>
#include <ostream>
#include <vector>

#include "VisualOdometer.h"
#include "matplotlibcpp.h"
#include "State.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;
namespace plt = matplotlibcpp;

void visualizeMotion(const cv::Mat &im,
                     const vector<KeyPoint> &kpsCurr,
                     const vector<KeyPoint> &kpsPrev,
                     const vector<DMatch> &matches,
                     cv::Mat &outIm)
{

    im.copyTo(outIm);
    for (auto m : matches)
    {
        cv::circle(outIm, kpsPrev[m.trainIdx].pt, 5, Scalar(0, 0, 255), 1);
        cv::circle(outIm, kpsCurr[m.queryIdx].pt, 5, Scalar(0, 255, 0), 1);

        cv::arrowedLine(outIm, kpsPrev[m.trainIdx].pt, kpsCurr[m.queryIdx].pt, Scalar(0, 0, 0));
        // cv::imshow("Matched Features", outIm);
        // waitKey();
    }
    return;
}

State VisualOdometer::runStep(cv::Mat currImg)
{
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
    for (auto match : matches)
    {
        if (match.distance < distThresh)
        {
            goodMatches.push_back(match);
            kps1m.push_back(kps1[match.trainIdx].pt);
            kps2m.push_back(kps2[match.queryIdx].pt);
            // cout << "Adding a match!" << endl;
        }
    }

    // Show Matches (for debugging)
    // cv::Mat outImg;
    // visualizeMotion(currImg, kps1, kps2, goodMatches, outImg);
    // imshow("Matched Features", outImg);
    // waitKey(0); // Wait for a keystroke in the window

    // Recover Pose
    cv::Mat E, R, t, mask;
<<<<<<< HEAD
<<<<<<< HEAD
    E = cv::findEssentialMat(kps1m, kps2m, this->k, RANSAC, 0.899, 1.0, mask);
    cout << "E: \n" << E << endl;
=======
    E = cv::findEssentialMat(kps1m, kps2m, this->k, RANSAC, 0.699, 1, mask);
>>>>>>> f761b41... Trajectory is better, but still broken.  Need to combine prior translations
    recoverPose(E, kps1m, kps2m, this->k, R, t, mask);
    // cout << "R:\n" << R << endl;
    // cout << "t:\n" << t << endl;
    
    arma::Mat<double> trans(4,4, arma::fill::eye);
    arma::Mat<double> temp1(reinterpret_cast<double*>(R.data), R.cols, R.rows);
    arma::Mat<double> temp2(reinterpret_cast<double*>(t.data), t.rows, t.cols);
<<<<<<< HEAD
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
=======
=======
    E = cv::findEssentialMat(kps1m, kps2m, this->k, RANSAC, 0.6, 1.0, mask);
    recoverPose(E, kps1m, kps2m, this->k, R, t, mask);
    // cout << "R:\n" << R << endl;
    // cout << "t:\n" << t << endl;

    arma::Mat<double> trans(4, 4, arma::fill::eye);
    arma::Mat<double> temp1(reinterpret_cast<double *>(R.data), R.cols, R.rows);
    arma::Mat<double> temp2(reinterpret_cast<double *>(t.data), t.rows, t.cols);
>>>>>>> 78fd042... changed generate dataset so images includes gt data
    // cout << "temp1:\n" << temp1.t() << endl;
    // cout << "temp2:\n" << temp2 << endl;

    trans.submat(0, 0, 2, 2) = temp1.t();
    trans.submat(0, 3, 2, 3) = -temp2;

    // prevTrans.submat(0,0,2,2) = prevState.rot.toRotMat();
    // prevTrans.submat(0, 3, 2, 3) = prevState.pos;

    // cout << "trans:\n" << trans << endl;
    arma::Col<double> p1(4, 1, arma::fill::zeros), p2(4, 1, arma::fill::ones);
    p1[3] = 1;

<<<<<<< HEAD
    p2 = arma::inv(trans) * p1;
    this->prevState.pos = p2.subvec(0,2) + this->prevState.pos;
>>>>>>> f761b41... Trajectory is better, but still broken.  Need to combine prior translations
=======
    prevTrans = arma::inv(prevTrans) * arma::inv(trans);
    p2 = prevTrans * p1;
    prevTrans = arma::inv(prevTrans);

    this->prevState.pos = p2.subvec(0, 2);
>>>>>>> 78fd042... changed generate dataset so images includes gt data
    this->prevImg = currImg;

    return this->prevState;
}

int main()
{
    std::string image_dir_path = "/home/jordan/Projects/self-driving-car/src/localization/test/images/";
<<<<<<< HEAD
<<<<<<< HEAD
    cv::Mat k({640, 0, 640, 0, 480, 480, 0, 0, 1});
    cout << k.dims << endl;
    cout << "in main k : \n" << k << endl;
=======
    cv::Mat k({400, 0, 400, 0, 400, 300, 0, 0, 1});
    cv::Mat img = imread(image_dir_path + "test_image_0.png", IMREAD_GRAYSCALE);
>>>>>>> f761b41... Trajectory is better, but still broken.  Need to combine prior translations

    k = k.reshape(1, {3,3});
    cout << k.dims << endl;
    cout << "in main k : \n" << k << endl;
    State s1 = State();
    VisualOdometer vo = VisualOdometer(k, img, s1);
    int maxNum = 500;
    int imNum = 1;
    vector<State> outputState(maxNum);
    vector<double> xOut(maxNum), yOut(maxNum), zOut(maxNum);
    while(true){
        std::string imName = image_dir_path + "test_image_" + to_string(imNum) + ".png";
        cout << "Image Name: " << imName << endl;
        try{
            img = imread(imName, IMREAD_GRAYSCALE);
            State s2 = vo.runStep(img);
            outputState.push_back(s2);
            xOut.push_back(s2.pos[0]);
            yOut.push_back(s2.pos[1]);
            zOut.push_back(s2.pos[2]);
            cout << "Predicted State: \n" << s2.pos << endl;
            // plt::plot(xOut, yOut, "--r");
            // plt::show();
        }
        catch(int e){
            break;
=======

    // Arguments
    bool showVideo = true;            // Before Running the simulation, display the input video
    bool runSim = false;              // Run the simulation
    auto IM_STYLE = IMREAD_GRAYSCALE; // The color style of images to run on simulation (IMREAD_GRAYSCALE or IMREAD_COLOR)
    int startNum = 0;               // First Image Number to run
    int endNum = 200;                 // Last Image Number to run

    // showVideo
    int imNum;
    if (showVideo)
    {
        imNum = startNum + 1;
        cv::namedWindow("Input Images", cv::WINDOW_FULLSCREEN);
        while (++imNum < endNum)
        {
            std::string imName = image_dir_path + "test_image_" + to_string(imNum) + ".png";
            cout << "Image Name: " << imName << endl;
            try
            {
                cv::Mat img = imread(imName, IMREAD_COLOR);
                cv::imshow("Input Images", img);
                waitKey(35);
            }
            catch (...)
            {
                cout << "\tImage not found! Assuming end of Dataset reached." << endl;
                break;
            }
>>>>>>> 78fd042... changed generate dataset so images includes gt data
        }
    }

    // Run Simulation
    if (runSim)
    {
        int maxSize = endNum - startNum + 1;
        vector<State> outputState(maxSize);
        vector<double> xOut(maxSize), yOut(maxSize), zOut(maxSize), xGt(maxSize), yGt(maxSize), zGt(maxSize);
        cv::Mat img = imread(image_dir_path + "test_image_" + to_string(startNum) + ".png", IMREAD_GRAYSCALE);
        cv::Mat k({400, 0, 400, 0, 400, 300, 0, 0, 1});

        k = k.reshape(1, {3, 3});
        State s1 = State();
        VisualOdometer vo = VisualOdometer(k, img, s1);
        imNum = startNum + 1;
        while (++imNum < endNum)
        {
            std::string imName = image_dir_path + "test_image_" + to_string(imNum) + ".png";
            cout << "Image Name: " << imName << endl;
            try
            {
                img = imread(imName, IMREAD_GRAYSCALE);
                State s2 = vo.runStep(img);
                outputState.push_back(s2);
                xOut.push_back(s2.pos[0]);
                yOut.push_back(s2.pos[1]);
                zOut.push_back(s2.pos[2]);
                cout << "Predicted State: \n"
                     << s2.pos << endl;
                // plt::plot(xOut, yOut, "--r");
                // plt::show();
            }
            catch (...)
            {
                cout << "\tImage not found! Assuming end of Dataset reached." << endl;
                break;
            }
        }

        plt::plot(xOut, yOut, "--b");
        plt::show();
    }
    // if(k == 's')
    // {
    //     imwrite("starry_night.png", img);
    // }
    return 0;
}

// cmake -D BUILD_TIFF=ON -D WITH_CUDA=ON -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=ON -D BUILD_PERF_TESTS=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/usr/local/src/opencv_contrib/modules /usr/local/src/opencv/
