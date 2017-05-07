/* 
 * File:   VisualPoseEstimator.hpp
 * Author: Jan Dufek
 *
 * Created on May 5, 2017, 2:59 PM
 */

#ifndef VISUALPOSEESTIMATOR_HPP
#define VISUALPOSEESTIMATOR_HPP

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

class VisualPoseEstimator {
public:
    VisualPoseEstimator(double, Point2d, Mat);
    VisualPoseEstimator(const VisualPoseEstimator& orig);
    virtual ~VisualPoseEstimator();

    void setDesiredPose(Mat);
    void findPose(Mat, Vec3f&, Mat&);
    void printPose();

private:

    // Minimum number of features
    const int minimumNumberFeatures = 500;

    // FAST parameters
    const int FASTThreshold = 20;
    const bool FASTNonmaxSuppression = true;

    // SURF parameters
    const int SURFMinHessian = 400;

    // Shi-Tomasi parameters
    const int ShiTomasiMaxCorners = 2000;
    const double ShiTomasiQualityLevel = 0.01;
    const double ShiTomasiMinDistance = 2;
    const int ShiTomasiBlockSize = 3;
    const bool ShiTomasiUseHarrisDetector = false;
    const double ShiTomasiK = 0.04;

    // Feature tracking parameters
    const Size trackingWindowSize = Size(35, 35);
    const int trackingMaxLevel = 3;
    const TermCriteria trackingTermCriteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
    const int trackingFlags = 0;
    const double trackingMinEigThreshold = 0.001;

    // Angle outlier rejection
    const int angleOutlierDifferenceThreshold = 10; // If the angle difference between the frames is higher than this threshold, the measurement will be rejected
    const int angleOutlierCounterThreshold = 3; // After this number of consecutive rejections, the new angle value will be accepted
    
    // Motion detection
    const int motionThreshold = 20; // If average distance of corresponding features is higher than this threshold, the camera have moved
    
    // Small angle detection
    const int smallAngleThreshold = 5; // If all three angles are less than threshold, the angle is considered small

    // Initial frame and features
    Mat initialFrame;
    vector<Point2f> features1;

    // Current features
    vector<Point2f> features2;

    // Camera intrinsic parameters
    double focalLength;
    Point2d principalPoint;
    Mat distortionCoefficients;

    // Current pose
    Mat rotationMatrix = Mat::zeros(3, 3, CV_64F);
    Vec3f eulerAngles;
    Mat translationMatrix = Mat::zeros(3, 1, CV_64F);
    
    // Measured pose
    Mat rotationMatrixMeasured;
    Vec3f eulerAnglesMeasured;
    Mat translationMatrixMeasured;
    
    // Mean of corresponding features distance
    double meanPointDistance;
    
    // Standard deviation of corresponding features distance
    double standardDeviationPointDistance;
    
    // Are points shifted
    bool pointsShifted;
    
    // Is small angle
    bool smallAngle;
    
    // Number of features
    int numberOfFeatures;
    
    // Redetection triggered
    bool redetection;
    
    // Is outlier
    bool outlier;

    bool isRotationMatrix(Mat&);
    Vec3f rotationMatrixToEulerAngles(Mat&);
    
    void detectFeaturesFAST(Mat&, vector<Point2f>&);
    void detectFeaturesSURF(Mat&, vector<Point2f>&);
    void detectFeaturesShiTomasi(Mat&, vector<Point2f>&);
    void detectFeatures(Mat&, vector<Point2f>&);
    
    void trackFeatures(Mat&, Mat&, vector<Point2f>&, vector<Point2f>&);
    
    void getPose(vector<Point2f>, vector<Point2f>, Mat&, Vec3f&, Mat&);
    
    void showFeatures(Mat&, vector<Point2f>&, string);

    // Number of times angle outlier was rejected in a row
    int angleOutlierRejectCounter = 0;
    bool isInlierAngle(Vec3f, Vec3f);
    
    double pointDistance(Point2f, Point2f);
    bool arePointsShifted(vector<Point2f>, vector<Point2f>);
    bool isAngleSmall(Vec3f);
    
    // Log
    ofstream logFile;
    string getCurrentTime();
    void log();
    
};

#endif /* VISUALPOSEESTIMATOR_HPP */

