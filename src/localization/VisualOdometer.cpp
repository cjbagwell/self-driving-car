#include <fstream>
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
                     const vector<KeyPoint> &currKps,
                     const vector<KeyPoint> &prevKps,
                     const vector<DMatch> &matches,
                     cv::Mat &outIm)
{

    im.copyTo(outIm);
    for (auto m : matches)
    {
        cv::circle(outIm, prevKps[m.trainIdx].pt, 5, Scalar(0, 0, 255), 1);
        cv::circle(outIm, currKps[m.queryIdx].pt, 5, Scalar(0, 255, 0), 1);

        cv::arrowedLine(outIm, prevKps[m.trainIdx].pt, currKps[m.queryIdx].pt, Scalar(0, 0, 0));
        // cv::imshow("Matched Features", outIm);
        // waitKey();
    }
    return;
}

State VisualOdometer::runStep(const cv::Mat &currImg)
{
    // Extract Features and Descriptors
    vector<cv::KeyPoint> currKps;
    cv::Mat currDes;
    detector->detectAndCompute(currImg, noArray(), currKps, currDes);

    // Find Matching Features
    vector<DMatch> matches;
    matcher->match(this->prevDes, currDes, matches);

    // Filter Matches
    const float thresh = 0.7;
    const int distThresh = 30;
    vector<DMatch> goodMatches;
    vector<Point2f> prevKpsm, currKpsm;
    for (auto match : matches)
    {
        if (match.distance < distThresh)
        {
            goodMatches.push_back(match);
            prevKpsm.push_back(prevKps[match.trainIdx].pt);
            currKpsm.push_back(currKps[match.queryIdx].pt);
            // cout << "Adding a match!" << endl;
        }
    }

    // Show Matches (for debugging)
    if (this->debug)
    {
        cv::Mat outImg;
        visualizeMotion(currImg, prevKps, currKps, goodMatches, outImg);
        cv::namedWindow("Matched Features - Debugging", cv::WINDOW_FULLSCREEN);
        imshow("Matched Features - Debugging", outImg);
        waitKey(1); // Wait for a keystroke in the window
    }

    // Recover Pose
    cv::Mat E, R, t, mask;
    E = cv::findEssentialMat(currKpsm, prevKpsm, this->k, RANSAC, 0.999, 1.0, cv::noArray());
    recoverPose(E, prevKpsm, currKpsm, this->k, R, t, cv::noArray());
    cout << "R:\n"
         << R << endl;
    cout << "t:\n"
         << t << endl;

    arma::Mat<double> trans(4, 4, arma::fill::eye);
    arma::Mat<double> temp1(reinterpret_cast<double *>(R.data), R.rows, R.cols);
    arma::Mat<double> temp2(reinterpret_cast<double *>(t.data), t.rows, t.cols);
    cout << "temp1:\n"
         << temp1 << endl;
    cout << "temp2:\n"
         << temp2 << endl;

    trans.submat(0, 0, 2, 2) = temp1.t();
    trans.submat(0, 3, 2, 3) = -temp2;

    // prevTrans.submat(0,0,2,2) = prevState.rot.toRotMat();
    // prevTrans.submat(0, 3, 2, 3) = prevState.pos;

    // cout << "trans:\n" << trans << endl;
    arma::Col<double> p1(4, 1, arma::fill::zeros), p2(4, 1, arma::fill::ones);
    p1[3] = 1;

    prevTrans = arma::inv(prevTrans) * arma::inv(trans);
    p2 = prevTrans * p1;
    prevTrans = arma::inv(prevTrans);

    this->prevState.pos = p2.subvec(0, 2);
    this->prevImg = currImg;
    this->prevDes = currDes;
    this->prevKps = currKps;

    return prevState;
}

int main()
{
    std::string imageDirPath = "/home/jordan/Projects/self-driving-car/src/localization/test/images/";

    // Arguments
    bool showVideo = false;       // Before Running the simulation, display the input video
    bool runSim = true;           // Run the simulation
    auto IM_STYLE = IMREAD_COLOR; // The color style of images to run on simulation (IMREAD_GRAYSCALE or IMREAD_COLOR)
    int startNum = 75;            // First Image Number to run
    int endNum = 125;             // Last Image Number to run

    // showVideo
    int imNum;
    if (showVideo)
    {
        imNum = startNum + 1;
        cv::namedWindow("Input Images", cv::WINDOW_FULLSCREEN);
        while (++imNum < endNum)
        {
            std::string imName = imageDirPath + "test_image_" + to_string(imNum) + ".png";
            cout << "Image Name: " << imName << endl;
            try
            {
                cv::Mat img = imread(imName, IM_STYLE);
                cv::imshow("Input Images", img);
                waitKey(1);
            }
            catch (...)
            {
                cout << "\tImage not found! Assuming end of Dataset reached." << endl;
                break;
            }
        }
    }

    // Run Simulation
    if (runSim)
    {
        int maxSize = endNum - startNum + 1;
        vector<State> outputState;
        vector<double> xOut, yOut, zOut, xGt, yGt, zGt, rollGt, pitchGt, yawGt;

        String gtFilePath = imageDirPath + "GT_Data.txt";
        std::ifstream infile(gtFilePath);
        if (infile.is_open())
        {
            while (infile)
            {
                string s;
                if (!getline(infile, s))
                    break;
                istringstream ss(s);
                vector<string> record;

                while (ss)
                {
                    string s;
                    if (!getline(ss, s, ','))
                        break;
                    record.push_back(s);
                }
                xGt.push_back(stod(record[1]));
                yGt.push_back(stod(record[2]));
                zGt.push_back(stod(record[3]));
                rollGt.push_back(stod(record[7]));
                pitchGt.push_back(stod(record[8]));
                yawGt.push_back(stod(record[9]));
                // cout << "x: " << stod(record[1]) << "\ty: " << stod(record[2]) << endl;
            }
            infile.close();
        }
        else
        {
            cout << "Error opening file";
        }

        cv::Mat img = imread(imageDirPath + "test_image_" + to_string(startNum) + ".png", IM_STYLE);
        cv::Mat k({400, 0, 400, 0, 400, 300, 0, 0, 1});
        k = k.reshape(1, {3, 3});
        State s1 = State();
        // s1.pos = {xGt[0],
        //           yGt[0],
        //           zGt[0]};
        arma::Col<double> eAngles = {rollGt[0], pitchGt[0], yawGt[0]};
        // s1.rot = Quaternion(eAngles, false);
        // cout << "init rot mat:\n" << s1.rot.toRotMat() << endl;
        VisualOdometer vo = VisualOdometer(k, img, s1);

        vector<arma::Mat<double>> outputTranslation;
        imNum = startNum;
        while (++imNum < endNum)
        {
            std::string imName = imageDirPath + "test_image_" + to_string(imNum) + ".png";
            cout << "Image Name: " << imName << endl;
            try
            {
                img = imread(imName, IM_STYLE);
                // arma::Mat<double> trans = vo.runStep(img);
                State s2 = vo.runStep(img);
                // outputState.push_back(s2);
                // outputTranslation.push_back(trans);
                xOut.push_back(s2.pos[0]);
                yOut.push_back(s2.pos[1]);
                zOut.push_back(s2.pos[2]);
                // cout << "Predicted State: \n"
                //      << s2.pos << endl;
                // plt::plot(xOut, yOut, "--r");
                // plt::show();
            }
            catch (...)
            {
                cout << "\tImage not found! Assuming end of Dataset reached." << endl;
                break;
            }
        }
        // arma::Mat<double> prevTrans = outputTranslation[0];
        // arma::Col<double> prevPos = {s1.pos[0], s1.pos[1], s1.pos[2], 1};
        // // cout << "prevPos:\n" << prevPos << endl;
        // xOut.push_back(prevPos[0]);
        // yOut.push_back(prevPos[1]);
        // zOut.push_back(prevPos[2]);
        // for(int i = startNum + 1; i < endNum-1; i++){
        //     cout << "loop i: " << i << endl;
        //     cout << "outputTrans: \n" << outputTranslation[i-1-startNum] << endl;
        //     prevPos = outputTranslation[i-1-startNum] * prevPos;
        //     cout << prevPos << endl;
        //     xOut.push_back(prevPos[0]);
        //     yOut.push_back(prevPos[1]);
        //     zOut.push_back(prevPos[2]);
        // }

        // arma::Mat<double> prevTrans = outputTranslation[0];
        // arma::Col<double> prevPos = {s1.pos[0], s1.pos[1], s1.pos[2], 1};
        // cout << "prevPos:\n" << prevPos << endl;
        // xOut.push_back(prevPos[0]);
        // yOut.push_back(prevPos[1]);
        // zOut.push_back(prevPos[2]);
        // for(int i = endNum-1; i > startNum; i--){
        //     prevPos = arma::inv(outputTranslation[])
        // }

        plt::plot(xOut, yOut, "r");
        xGt = vector<double>(xGt.begin() + startNum, xGt.begin() + endNum);
        yGt = vector<double>(yGt.begin() + startNum, yGt.begin() + endNum);
        zGt = vector<double>(zGt.begin() + startNum, zGt.begin() + endNum);
        plt::plot(xGt, yGt, "--b");
        plt::plot(vector<double>(xGt.begin(), xGt.begin() + 1), vector<double>(yGt.begin(), yGt.begin() + 1), "r*");
        plt::show();
    }
    // if(k == 's')
    // {
    //     imwrite("starry_night.png", img);
    // }
    return 0;
}

// cmake -D BUILD_TIFF=ON -D WITH_CUDA=ON -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=ON -D BUILD_PERF_TESTS=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/usr/local/src/opencv_contrib/modules /usr/local/src/opencv/
