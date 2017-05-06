#include "VisualPoseEstimator.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

    // Initialize video capture
    VideoCapture videoCapture(1);
    
    // Initialize camera parameters
    double focalLength = 1.2947690218730506e+03;
    Point principalPoint = Point(640, 400);
    Mat distortionCoefficients = (Mat_<double>(1, 5) << 7.1921671001332552e-02, -3.0711748457869636e-01, 0, 0, 2.2349323234306143e-01);

    // Initialize visual pose estimator
    VisualPoseEstimator * visualPoseEstimator = new VisualPoseEstimator(focalLength, principalPoint, distortionCoefficients);

    // Initialize initial frame
    Mat initialFrame;
    
    // Get current frame (get it twice since the first one might be bad)
    videoCapture >> initialFrame;
    videoCapture >> initialFrame;
    
    // Fix current pose
    visualPoseEstimator->setDesiredPose(initialFrame);
    
    // Initialize current frame
    Mat frame;

    // While ESC key is not pressed
    while (waitKey(1) != 27) {

        // Load current frame
        videoCapture >> frame;

        // Initialize euler angles and translation matrix         
        Vec3f eulerAngles;
        Mat translationMatrix;
        
        // Find current pose
        visualPoseEstimator->findPose(frame, eulerAngles, translationMatrix);
        
        // Print current pose
        visualPoseEstimator->printPose();

    }
    
    delete visualPoseEstimator;

    return 0;
}