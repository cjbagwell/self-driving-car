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
    return maxDiff < 0.000001 || (maxDiff - PI) < 0.000001 || (maxDiff + PI) < 0.000001;
}

TEST(RotationsTests, Angles_Normalise_Calculation){
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
}
TEST(RotationsTests, Skew_Matrix_Calculation){
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
}    

TEST(RotationsTests, Quaternions_Euler_Angles_Conversions){
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
    ASSERT_TRUE(testQuat == eQuat || -testQuat == eQuat) << eQuat;

    eAngles = Col<double> ({PI/2, 0, 0});
    eQuat = Quaternion(eAngles, false);
    testQuat = Quaternion(sqrt(2)/2, sqrt(2)/2, 0, 0);
    EXPECT_TRUE(testQuat == eQuat || -testQuat == eQuat) << "expected:\n" << testQuat << "\nwas:\n" << eQuat;
    EXPECT_TRUE(isEqual(eAngles, eQuat.toEulerAngles())) << "expected:\n" << eAngles << "\nwas:\n" << eQuat.toEulerAngles();

    eAngles = Col<double> ({0, PI/2, 0});
    eQuat = Quaternion(eAngles, false);
    testQuat = Quaternion(sqrt(2)/2, 0, sqrt(2)/2, 0);
    EXPECT_TRUE(testQuat == eQuat || -testQuat == eQuat) << "expected:\n" << testQuat << "\nwas:\n" << eQuat;
    EXPECT_TRUE(isEqual(eAngles, eQuat.toEulerAngles())) << "expected:\n" << eAngles << "\nwas:\n" << eQuat.toEulerAngles();

    eAngles = Col<double>({0, 0, PI/2});
    eQuat = Quaternion(eAngles, false);
    testQuat = Quaternion(sqrt(2)/2, 0, 0, sqrt(2)/2);
    EXPECT_TRUE(testQuat == eQuat || -testQuat == eQuat) << "expected:\n" << testQuat << "\nwas:\n" << eQuat;
    EXPECT_TRUE(isEqual(eAngles, eQuat.toEulerAngles())) << "expected:\n" << eAngles << "\nwas:\n" << eQuat.toEulerAngles();

    eAngles = Col<double>({PI/4, PI/2, -PI/4});
    eQuat = Quaternion(eAngles, false);
    testQuat = Quaternion(sqrt(2)/2, 0, sqrt(2)/2, 0);
    EXPECT_TRUE(testQuat == eQuat || -testQuat == eQuat) << "expected:\n" << testQuat << "\nwas:\n" << eQuat;
    EXPECT_TRUE(isEqual(eAngles, eQuat.toEulerAngles())) << "expected:\n" << eAngles << "\nwas:\n" << eQuat.toEulerAngles();

    eAngles = Col<double>({PI/6, PI/4, PI/3});
    eQuat = Quaternion(eAngles, false);
    testQuat = Quaternion(0.7233174, 0.3919038, 0.2005621, 0.5319757);
    EXPECT_TRUE(testQuat == eQuat || -testQuat == eQuat) << "expected:\n" << testQuat << "\nwas:\n" << eQuat;
    EXPECT_TRUE(isEqual(eAngles, eQuat.toEulerAngles())) << "expected:\n" << eAngles << "\nwas:\n" << eQuat.toEulerAngles();
}

TEST(RotationsTests, Quaternions_Axis_Angles_Conversions){
    // Test Axis Angles Quaternions
    Col<double> aAngles = Col<double>({PI/2, 0, 0});
    Quaternion aQuat(aAngles, true);
    Quaternion testQuat = Quaternion(sqrt(2)/2, sqrt(2)/2, 0, 0);
    EXPECT_TRUE(testQuat == aQuat) << "expected:\n" << testQuat << "\nwas:\n" << aQuat;
    EXPECT_TRUE(isEqual(aAngles, aQuat.toAxisAngles())) << "expected:\n" << aAngles << "\nwas:\n" << aQuat.toAxisAngles();

    aAngles = Col<double>({0, PI/2, 0});
    aQuat = Quaternion(aAngles, true);
    testQuat = Quaternion(sqrt(2)/2, 0, sqrt(2)/2, 0);
    EXPECT_TRUE(testQuat == aQuat) << "expected:\n" << testQuat << "\nwas:\n" << aQuat;
    EXPECT_TRUE(isEqual(aAngles, aQuat.toAxisAngles())) << "expected:\n" << aAngles << "\nwas:\n" << aQuat.toAxisAngles();


    aAngles = Col<double>({0, 0, PI/2});
    aQuat = Quaternion(aAngles, true);
    testQuat = Quaternion(sqrt(2)/2, 0, 0, sqrt(2)/2);
    EXPECT_TRUE(testQuat == aQuat) << "expected:\n" << testQuat << "\nwas:\n" << aQuat;
    EXPECT_TRUE(isEqual(aAngles, aQuat.toAxisAngles())) << "expected:\n" << aAngles << "\nwas:\n" << aQuat.toAxisAngles();
}

TEST(RotationsTests, Quaternion_Rotation_Matrix_Conversions){
    Mat<double> testRotMat(3,3,fill::zeros);
    testRotMat(0,0) = 1;
    testRotMat(1,2) = -1;
    testRotMat(2,1) = 1;
    Col<double> eAngles = Col<double>({PI/2, 0, 0});
    Quaternion rotMatQuat(eAngles, true);
    EXPECT_TRUE(isEqual(testRotMat, rotMatQuat.toRotMat())) << "Expected:\n" <<
        testRotMat << "\nwas:\n" << rotMatQuat.toRotMat();

    testRotMat(0,0) = 0.3535534;
    testRotMat(0,1) = -0.6123725;
    testRotMat(0,2) = 0.7071068;
    testRotMat(1,0) = 0.9267767;
    testRotMat(1,1) = 0.1268265;
    testRotMat(1,2) = -0.3535534;
    testRotMat(2,0) = 0.1268265;
    testRotMat(2,1) = 0.7803301;
    testRotMat(2,2) = 0.6123725;
    eAngles = Col<double>({PI/6, PI/4, -PI/3});
    rotMatQuat = Quaternion(eAngles, false);
    EXPECT_TRUE(isEqual(testRotMat, rotMatQuat.toRotMat())) << "Expected:\n" <<
        testRotMat << "\nwas:\n" << rotMatQuat.toRotMat();
}

TEST(RotationsTests, Quaternion_Multiplication){
    Quaternion q1 = Quaternion(sqrt(2)/2, sqrt(2)/2, 0, 0);
    Quaternion q2 = Quaternion(sqrt(2)/2, 0, sqrt(2)/2, 0);
    Quaternion qExpected = Quaternion(0.5, 0.5, 0.5, 0.5);
    Quaternion qActual = q1 * q2;
    EXPECT_TRUE(qActual == qExpected) << "Expected:\n" << qExpected << "\nwas:\n" << qActual;

    q2 = Quaternion(0.7616657,0.2406501, 0.3609751, 0.4813002);
    qExpected = Quaternion(0.3684136638, 0.7087442990, -0.08508269415, 0.5955785762);
    qActual = q1 * q2;
    EXPECT_TRUE(qActual == qExpected) << "Expected:\n" << qExpected << "\nwas:\n" << qActual;
}