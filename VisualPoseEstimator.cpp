/* 
 * File:   VisualPoseEstimator.cpp
 * Author: Jan Dufek
 * 
 * Created on May 5, 2017, 2:59 PM
 */

#include "VisualPoseEstimator.hpp"

VisualPoseEstimator::VisualPoseEstimator(double focalLength, Point2d principalPoint, Mat distortionCoefficients) {

    // Initialize camera intrinsic parameters
    this->focalLength = focalLength;
    this->principalPoint = principalPoint;
    this->distortionCoefficients = distortionCoefficients;

    // Initialize log
    logFile.open("logs/features/" + getCurrentTime() + ".txt");

}

VisualPoseEstimator::VisualPoseEstimator(const VisualPoseEstimator& orig) {
}

VisualPoseEstimator::~VisualPoseEstimator() {

    // Close log
    logFile.close();

}

/**
 * Returns true if the given matrix a rotation matrix.
 * 
 * @param matrix input matrix
 * @return 
 */
bool VisualPoseEstimator::isRotationMatrix(Mat& matrix) {

    Mat matrixTranspose;
    transpose(matrix, matrixTranspose);
    Mat identity = Mat::eye(3, 3, matrix.type());
    return norm(identity, matrixTranspose * matrix) < 1e-6;

}

/**
 * Converts rotation matrix to euler angles.
 * The euler angles are given in the order x, y, and z.
 * 
 * @param rotationMatrix input rotation matrix
 * @return angles in degrees
 */
Vec3f VisualPoseEstimator::rotationMatrixToEulerAngles(Mat& rotationMatrix) {

    // Check if the matrix is rotation matrix
    assert(isRotationMatrix(rotationMatrix));

    // Compute sy
    double sy = sqrt(pow(rotationMatrix.at<double>(0, 0), 2) + pow(rotationMatrix.at<double>(1, 0), 2));

    // Check if the matrix is singular
    bool singular = sy < 1e-6;

    // Rotation angles
    double x, y, z;

    // Compute angles
    if (!singular) {
        x = atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));
        y = atan2(-rotationMatrix.at<double>(2, 0), sy);
        z = atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));
    } else {
        x = atan2(-rotationMatrix.at<double>(1, 2), rotationMatrix.at<double>(1, 1));
        y = atan2(-rotationMatrix.at<double>(2, 0), sy);
        z = 0;
    }

    // Return angles in degrees
    return Vec3f((x * 180) / CV_PI, (y * 180) / CV_PI, (z * 180) / CV_PI);
}

/**
 * Detect features using FAST algorithm.
 * 
 * @param image
 * @param features
 */
void VisualPoseEstimator::detectFeaturesFAST(Mat& image, vector<Point2f>& features) {
    vector<KeyPoint> keypoints;
    FAST(image, keypoints, FASTThreshold, FASTNonmaxSuppression);
    KeyPoint::convert(keypoints, features, vector<int>());
}

/**
 * Detect features using SURF algorithm.
 * 
 * @param image
 * @param features
 */
void VisualPoseEstimator::detectFeaturesSURF(Mat& image, vector<Point2f>& features) {
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(SURFMinHessian);
    Mat descriptors;
    vector<KeyPoint> keypoints;
    detector->detectAndCompute(image, Mat(), keypoints, descriptors);
    KeyPoint::convert(keypoints, features, vector<int>());
}

/**
 * Detect features using Shi-Tomasi algorithm.
 * 
 * @param image
 * @param features
 */
void VisualPoseEstimator::detectFeaturesShiTomasi(Mat& image, vector<Point2f>& features) {

    // Detect features
    goodFeaturesToTrack(image, features, ShiTomasiMaxCorners, ShiTomasiQualityLevel, ShiTomasiMinDistance, Mat(), ShiTomasiBlockSize, ShiTomasiUseHarrisDetector, ShiTomasiK);

}

/**
 * Detects features in the give frame.
 * 
 * @param image
 * @param features
 */
void VisualPoseEstimator::detectFeatures(Mat& image, vector<Point2f>& features) {

    //    detectFeaturesFAST(image, features);
    //    detectFeaturesSURF(image, features);
    detectFeaturesShiTomasi(image, features);

}

/**
 * Tracks features1 from image1 to image2 using optic flow.
 * Lucas-Kanade method is used.
 * 
 * @param image1
 * @param image2
 * @param features1
 * @param features2
 */
void VisualPoseEstimator::trackFeatures(Mat& image1, Mat& image2, vector<Point2f>& features1, vector<Point2f>& features2) {

    vector<float> error;
    vector<uchar> status;

    calcOpticalFlowPyrLK(image1, image2, features1, features2, status, error, trackingWindowSize, trackingMaxLevel, trackingTermCriteria, trackingFlags, trackingMinEigThreshold);

    // Delete features for which tracking failed or went out of the frame
    int offset = 0;

    for (int i = 0; i < status.size(); i++) {

        // Current feature
        Point2f feature = features2.at(i - offset);

        // If the feature went out of view, invalidate it
        if (feature.x < 0 || feature.y < 0) {
            status.at(i) = 0;
        }

        // Check if the current feature is valid (corresponding feature was found)
        if (status.at(i) == 0) {

            // Feature is invalid, erase it
            features1.erase(features1.begin() + (i - offset));
            features2.erase(features2.begin() + (i - offset));

            // Increase offset
            offset++;

        }

    }

    // Save number of features
    numberOfFeatures = features2.size();
}

/**
 * Get 3D pose from 2D point correspondences.
 * The 3D pose is returned as rotation matrix, euler angles, and translation matrix.
 * 
 * @param points1
 * @param points2
 * @param rotationMatrix
 * @param eulerAngles
 * @param translationMatrix
 */
void VisualPoseEstimator::getPose(vector<Point2f> points1, vector<Point2f> points2, Mat& rotationMatrix, Vec3f& eulerAngles, Mat& translationMatrix) {

    // Undistort points
    Mat K = Mat::zeros(3, 3, CV_64F);
    K.at<double>(0, 0) = focalLength;
    K.at<double>(1, 1) = focalLength;
    K.at<double>(2, 2) = 1;
    K.at<double>(0, 2) = principalPoint.x;
    K.at<double>(1, 2) = principalPoint.y;
    undistortPoints(points1, points1, K, distortionCoefficients, noArray(), K);
    undistortPoints(points2, points2, K, distortionCoefficients, noArray(), K);

    // Get new focal length and principal point
    double newFocalLength = K.at<double>(0, 0);
    Point newPrincipalPoint = Point(K.at<double>(0, 2), K.at<double>(1, 2));

    // Find essential matrix
    Mat mask;
    Mat essentialMatrix = findEssentialMat(points1, points2, newFocalLength, newPrincipalPoint, RANSAC, 0.999, 3, mask);

    // Correct matches
    correctMatches(essentialMatrix, points1, points2, points1, points2);

    // Find rotation and translation matrices
    recoverPose(essentialMatrix, points1, points2, rotationMatrix, translationMatrix, newFocalLength, newPrincipalPoint, mask);

    // Decompose rotation matrix into euler angles
    eulerAngles = rotationMatrixToEulerAngles(rotationMatrix);
}

/**
 * Show given features in an image.
 * 
 * @param frame
 * @param features
 * @param windowName
 */
void VisualPoseEstimator::showFeatures(Mat& frame, vector<Point2f>& features, string windowName) {

    // For each feature
    for (Point point : features) {

        // Draw a circle
        circle(frame, point, 3, Scalar(0, 255, 0));
    }

    // Show frame
    imshow(windowName, frame);

}

/**
 * Print rotation matrix, euler angles, and translation matrices.
 * 
 */
void VisualPoseEstimator::printPose() {

    //        cout << rotationMatrix << endl;
    cout << eulerAngles << endl;
    cout << translationMatrix << endl;

}

/**
 * Verify if the new angle is outlier.
 * 
 * @param eulerAngles
 * @param newEulerAngles
 * @return 
 */
bool VisualPoseEstimator::isInlierAngle(Vec3f eulerAngles, Vec3f newEulerAngles) {

    // Check all three angles
    for (int i = 0; i < 3; i++) {

        // If one of the angles is higher than threshold, reject
        if (abs(eulerAngles[i] - newEulerAngles[i]) > angleOutlierDifferenceThreshold) {

            // Increase reject counter
            angleOutlierRejectCounter++;

            // If rejected multiple times in a row, accept
            if (angleOutlierRejectCounter > angleOutlierCounterThreshold) {

                // Reset reject counter
                angleOutlierRejectCounter = 0;

                outlier = false;
                return true;
            }

            outlier = true;
            return false;
        }
    }

    // Reset reject counter
    angleOutlierRejectCounter = 0;

    // All of the angles are smaller than threshold, accept
    outlier = false;
    return true;
}

/**
 * Distance between two 2D points.
 * 
 * @param point1
 * @param point2
 * @return 
 */
double VisualPoseEstimator::pointDistance(Point2f point1, Point2f point2) {
    return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
}

/**
 * Returns true if the features moved significantly.
 * 
 * @param features1
 * @param features2
 * @return 
 */
bool VisualPoseEstimator::arePointsShifted(vector<Point2f> features1, vector<Point2f> features2) {

    // Mean distance of points
    meanPointDistance = 0;

    // Sum of point distances
    for (int i = 0; i < features1.size(); i++) {
        meanPointDistance += pointDistance(features1[i], features2[i]);
    }

    // Compute mean
    meanPointDistance /= features1.size();
    
    // Compute standard deviation
    standardDeviationPointDistance = 0;
    
    for (int i = 0; i < features1.size(); i++) {
        standardDeviationPointDistance += pow(pointDistance(features1[i], features2[i]) - meanPointDistance, 2);
    }
    
    standardDeviationPointDistance = sqrt(standardDeviationPointDistance / features1.size());

    // Check if the mean distance is more than threshold
    pointsShifted = meanPointDistance > motionThreshold;

    return pointsShifted;
}

/**
 * Are the euler angles small to be considered almost 0.
 * 
 * @param eulerAngles
 * @return 
 */
bool VisualPoseEstimator::isAngleSmall(Vec3f eulerAngles) {

    // Check all three angles
    for (int i = 0; i < 3; i++) {

        // If one of them is higher than threshold, it is big angle
        if (abs(eulerAngles[i]) > smallAngleThreshold) {

            smallAngle = false;
            return false;

        }
    }

    // All of the angles are smaller than threshold, it is small angle
    smallAngle = true;
    return true;

}

/**
 * Returns current time stamp in string %Y%m%d%H%M%S format.
 * 
 * @return 
 */
string VisualPoseEstimator::getCurrentTime() {

    time_t rawTime;
    time(&rawTime);
    struct tm * localTime;
    localTime = localtime(&rawTime);
    char currentTime[40];
    strftime(currentTime, 40, "%Y%m%d%H%M%S", localTime);
    string currentTimeString(currentTime);

    return currentTimeString;
}

void VisualPoseEstimator::log() {

    // Current time
    logFile << getCurrentTime();
    logFile << " ";

    // Euler angles (modified)
    logFile << eulerAngles[0];
    logFile << " ";
    logFile << eulerAngles[1];
    logFile << " ";
    logFile << eulerAngles[2];
    logFile << " ";

    // Translation matrix (modified)
    logFile << translationMatrix.at<double>(0, 0);
    logFile << " ";
    logFile << translationMatrix.at<double>(1, 0);
    logFile << " ";
    logFile << translationMatrix.at<double>(2, 0);
    logFile << " ";

    // Rotation matrix (modified)
    logFile << rotationMatrix.at<double>(0, 0);
    logFile << " ";
    logFile << rotationMatrix.at<double>(0, 1);
    logFile << " ";
    logFile << rotationMatrix.at<double>(0, 2);
    logFile << " ";
    logFile << rotationMatrix.at<double>(1, 0);
    logFile << " ";
    logFile << rotationMatrix.at<double>(1, 1);
    logFile << " ";
    logFile << rotationMatrix.at<double>(1, 2);
    logFile << " ";
    logFile << rotationMatrix.at<double>(2, 0);
    logFile << " ";
    logFile << rotationMatrix.at<double>(2, 1);
    logFile << " ";
    logFile << rotationMatrix.at<double>(2, 2);
    logFile << " ";

    // Euler angles (measured)
    logFile << eulerAnglesMeasured[0];
    logFile << " ";
    logFile << eulerAnglesMeasured[1];
    logFile << " ";
    logFile << eulerAnglesMeasured[2];
    logFile << " ";

    // Translation matrix (measured)
    logFile << translationMatrixMeasured.at<double>(0, 0);
    logFile << " ";
    logFile << translationMatrixMeasured.at<double>(1, 0);
    logFile << " ";
    logFile << translationMatrixMeasured.at<double>(2, 0);
    logFile << " ";

    // Rotation matrix (measured)
    logFile << rotationMatrixMeasured.at<double>(0, 0);
    logFile << " ";
    logFile << rotationMatrixMeasured.at<double>(0, 1);
    logFile << " ";
    logFile << rotationMatrixMeasured.at<double>(0, 2);
    logFile << " ";
    logFile << rotationMatrixMeasured.at<double>(1, 0);
    logFile << " ";
    logFile << rotationMatrixMeasured.at<double>(1, 1);
    logFile << " ";
    logFile << rotationMatrixMeasured.at<double>(1, 2);
    logFile << " ";
    logFile << rotationMatrixMeasured.at<double>(2, 0);
    logFile << " ";
    logFile << rotationMatrixMeasured.at<double>(2, 1);
    logFile << " ";
    logFile << rotationMatrixMeasured.at<double>(2, 2);
    logFile << " ";

    // Mean point distance
    logFile << meanPointDistance;
    logFile << " ";
    
    // Standard deviation of point distance
    logFile << standardDeviationPointDistance;
    logFile << " ";
    
    // Are points shifted
    logFile << pointsShifted;
    logFile << " ";

    // Is angle small
    logFile << smallAngle;
    logFile << " ";

    // Number of features
    logFile << numberOfFeatures;
    logFile << " ";

    // Redection
    logFile << redetection;
    logFile << " ";

    // Is outlier
    logFile << outlier;
    logFile << " ";

    // Outlier counter
    logFile << angleOutlierRejectCounter;
    logFile << " ";

    logFile << "\n";
}

/**
 * Set current pose as a base line.
 * The output of findPose will be relative to this baseline.
 * 
 * @param frame
 */
void VisualPoseEstimator::setDesiredPose(Mat frame) {

    // Covert initial frame to grayscale
    cvtColor(frame, initialFrame, COLOR_BGR2GRAY);

    // Detect features in the initial frame
    detectFeatures(initialFrame, features1);

    // Show initial features
    showFeatures(frame, features1, "Initial Features");

}

/**
 * Find pose for current frame relative to the pose set by setDesiredPose function.
 * 
 * camera moved right -> x is negative (go left)
 * camera moved left -> x is positive (go right)
 * 
 * camera moved up -> y is positive (go down)
 * camera moved down -> y is negative (go up)
 * 
 * camera moved forward -> z is negative (go backward)
 * camera moved backward -> z is positive (go forward)
 * 
 * camera turned right -> y is negative (yaw left)
 * camera turned left -> y is positive (yaw right)
 * 
 * camera turned down -> x is positive (tilt up)
 * camera turned up -> x is negative (tilt down)
 * 
 * camera turned clockwise -> z is negative (roll counterclockwise)
 * camera turned counterclockwise -> z is positive (roll clockwise)
 * 
 * @param frame
 * @param outputEulerAngles 
 * @param outputTranslationMatrix
 */
void VisualPoseEstimator::findPose(Mat frame, Vec3f& outputEulerAngles, Mat& outputTranslationMatrix) {

    // Initialize current frame
    Mat currentFrame;

    // Convert to grayscale
    cvtColor(frame, currentFrame, COLOR_BGR2GRAY);

    // Track features
    trackFeatures(initialFrame, currentFrame, features1, features2);

    // Show current features
    showFeatures(frame, features2, "Current Features");

    // Get pose
    getPose(features1, features2, rotationMatrixMeasured, eulerAnglesMeasured, translationMatrixMeasured);

    // Reject outliers
    if (isInlierAngle(eulerAngles, eulerAnglesMeasured)) {

        // Update rotation matrix
        rotationMatrix = rotationMatrixMeasured;

        // Update euler angles
        eulerAngles = eulerAnglesMeasured;

        // If angle is small but points are shifted, we have translation difference
        if (isAngleSmall(eulerAngles) && arePointsShifted(features1, features2)) {

            // Update translation
            translationMatrix = translationMatrixMeasured;
        } else {

            // The difference is due to angle, set translation to 0
            translationMatrix = Mat::zeros(3, 1, CV_64F);

        }

    } else {
        cout << "Outlier: " << eulerAnglesMeasured << endl;
    }

    // If the number of features drops bellow certain level, detect features again
    if (features2.size() < minimumNumberFeatures) {
        cout << "Feature redetection." << endl;
        redetection = true;
        detectFeatures(initialFrame, features1);
    }

    // Set output
    outputEulerAngles = eulerAngles;
    outputTranslationMatrix = translationMatrix;

    // Log
    log();
    
    // Unset redetection flag
    redetection = false;
}