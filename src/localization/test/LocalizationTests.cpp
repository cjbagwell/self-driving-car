#include<iostream>
#include<ostream> 
#include<vector>
#include<gtest/gtest.h>
#include<armadillo>
#include<math.h>
#include "../rotations.h"


using namespace std;
using namespace arma;

bool isEqual(const Mat<double>& m1, const Mat<double>& m2){
    if(arma::max(arma::max(arma::abs(m1 - m2)) < 0.000001)){
        return true;
    }
    return false;
}

bool isEqual(const Col<double>& c1, const Col<double>& c2){
    Col<double> diff = c1 - c2;
    diff = angleNormalise(diff);
    double maxDiff = arma::max(arma::abs(diff));
    cout << "maxDiff: " << maxDiff << endl;
    return maxDiff < 0.000001 || (maxDiff - PI) < 0.000001 || (maxDiff + PI) < 0.000001;
}

TEST(LocalizationTests, RotationTests){
    // Test wrap angle
    double ang1 = 5;
    double ang2 = -5;
    double ang3 = 10;
    ASSERT_TRUE(angleNormalise(ang1) < PI);
    ASSERT_TRUE(angleNormalise(ang2) > -PI);
    ASSERT_TRUE(angleNormalise(ang3) < PI);
    
    Col<double> angs = {ang1, ang2, ang3};
    Col<double> outAngs = angleNormalise(angs);
    bool isCorrect = true;
    for(auto ang : outAngs){
        if(ang < -PI || ang > PI){
            isCorrect = false;
            break;
        }
    }
    ASSERT_TRUE(isCorrect);
    
    // Test Skew Symetric
    Col<double> v({1.0, 2.0, 3.0});
    Mat<double> m(3,3, fill::zeros);
    m(0,1) = -3.0;
    m(0,2) = 2.0;
    m(1,0) = 3.0;
    m(1,2) = -1.0;
    m(2,0) = -2.0;
    m(2,1) = 1.0;
    Mat<double> mTest = skewSemetric(v);
    ASSERT_TRUE(isEqual(m, mTest));


    // Test Basic Quaternion Constructors
    Quaternion testQuat(1, 0, 0, 0);
    Quaternion testQuat2;
    Row<double> quatVec({1, 0, 0, 0});
    Quaternion testQuat3(quatVec);
    ASSERT_TRUE(testQuat == testQuat2);
    ASSERT_TRUE(testQuat == testQuat3);

    // Test Euler Angles Quaternions
    Col<double> eAngles({0, 0, 0}); // Euler Angles
    Quaternion eQuat(eAngles, false);
    ASSERT_TRUE(testQuat == eQuat) << eQuat;

    eAngles = Col<double> ({PI/2, 0, 0});
    eQuat = Quaternion(eAngles, false);
    testQuat = Quaternion(sqrt(2)/2, sqrt(2)/2, 0, 0);
    EXPECT_TRUE(testQuat == eQuat) << "90 deg x rotation -- expected:" << testQuat << "\nwas:" << eQuat;
    EXPECT_TRUE(isEqual(eAngles, eQuat.toEulerAngles())) << "x rot -> to eAngles\nexpected:" << eAngles << "\nwas:" << eQuat.toEulerAngles();

    eAngles = Col<double> ({0, PI/2, 0});
    eQuat = Quaternion(eAngles, false);
    testQuat = Quaternion(sqrt(2)/2, 0, sqrt(2)/2, 0);
    EXPECT_TRUE(testQuat == eQuat) << "90 deg y rotation -- expected:" << testQuat << "\nwas:" << eQuat;
    EXPECT_TRUE(isEqual(eAngles, eQuat.toEulerAngles())) << "y rot -> to eAngles\nexpected:" << eAngles << "\nwas:" << eQuat.toEulerAngles();

    eAngles = Col<double>({0, 0, PI/2});
    eQuat = Quaternion(eAngles, false);
    testQuat = Quaternion(sqrt(2)/2, 0, 0, sqrt(2)/2);
    EXPECT_TRUE(testQuat == eQuat) << "90 deg z rotation -- expected:" << testQuat << "\nwas:" << eQuat;
    EXPECT_TRUE(isEqual(eAngles, eQuat.toEulerAngles())) << "z rot -> to eAngles\nexpected:" << eAngles << "\nwas:" << eQuat.toEulerAngles();

    // Test Axis Angles Quaternions
    Col<double> aAngles = Col<double>({PI/2, 0, 0});
    Quaternion aQuat(aAngles, true);
    testQuat = Quaternion(sqrt(2)/2, sqrt(2)/2, 0, 0);
    EXPECT_TRUE(testQuat == aQuat) << "90 deg x rotation -- expected:" << testQuat << "\nwas:" << aQuat;
    EXPECT_TRUE(isEqual(aAngles, aQuat.toAxisAngles())) << "x rot -> to aAngles\nexpected:" << aAngles << "\nwas:" << aQuat.toAxisAngles();

    aAngles = Col<double>({0, PI/2, 0});
    aQuat = Quaternion(aAngles, true);
    testQuat = Quaternion(sqrt(2)/2, 0, sqrt(2)/2, 0);
    EXPECT_TRUE(testQuat == aQuat) << "90 deg y rotation -- expected:" << testQuat << "\nwas:" << aQuat;
    EXPECT_TRUE(isEqual(aAngles, aQuat.toAxisAngles())) << "y rot -> to aAngles\nexpected:" << aAngles << "\nwas:" << aQuat.toAxisAngles();


    aAngles = Col<double>({0, 0, PI/2});
    aQuat = Quaternion(aAngles, true);
    testQuat = Quaternion(sqrt(2)/2, 0, 0, sqrt(2)/2);
    EXPECT_TRUE(testQuat == aQuat) << "90 deg z rotation -- expected:" << testQuat << "\nwas:" << aQuat;
    EXPECT_TRUE(isEqual(aAngles, aQuat.toAxisAngles())) << "z rot -> to aAngles\nexpected:" << aAngles << "\nwas:" << aQuat.toAxisAngles();


}    
