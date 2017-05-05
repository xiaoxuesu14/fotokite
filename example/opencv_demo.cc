/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>

#include "opencv2/opencv.hpp"
#include <math.h>
#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/getopt.h"

#include "common/homography.h"
#include "curl/curl.h"
#include <ctime>

#include "Fotokite.hpp" 

using namespace std;
using namespace cv;

void rot2euler(matd_t *R) // http://nghiaho.com/?page_id=846
{
    double temp = MATD_EL(R,2,1)*MATD_EL(R,2,1)+MATD_EL(R,2,2)*MATD_EL(R,2,2);
    temp = sqrt(temp);
    double theta_x = atan2(MATD_EL(R,2,1),MATD_EL(R,2,2))/3.1415926*180;
    double theta_y = atan2(-MATD_EL(R,2,0),temp)/3.1415926*180;
    double theta_z = atan2(MATD_EL(R,1,0), MATD_EL(R,0,0))/3.1415926*180;
    cout<<"theta x: "<<theta_x<<",\ttheta_y: "<<theta_y<<",\ttheta_z: "<<theta_z<<",\tx: "<<MATD_EL(R,0,3)<<",\ty: "<<MATD_EL(R,1,3)<<",\tz: "<<MATD_EL(R,2,3)<<endl;
}

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    // VideoCapture cap("rtsp://192.168.2.202/z3-1.mp4");
    VideoCapture cap(1);
  
    double fx = 1.002560636338565e+03; // Mac webcam
    double fy = 1.003227769508857e+03;
    double cx = 6.360588037637018e+02;
    double cy = 3.490447569094387e+02;
    // double fx = 8.846756775004567e+02; // FK GoPro
    // double fy = 8.907163863307561e+02;
    // double cx = 9.527944439036946e+02;
    // double cy = 5.362323663691869e+02;
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11"))
        tf = tag36h11_create();
    else if (!strcmp(famname, "tag36h10"))
        tf = tag36h10_create();
    else if (!strcmp(famname, "tag36artoolkit"))
        tf = tag36artoolkit_create();
    else if (!strcmp(famname, "tag25h9"))
        tf = tag25h9_create();
    else if (!strcmp(famname, "tag25h7"))
        tf = tag25h7_create();
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }
    tf->black_border = getopt_get_int(getopt, "border");

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    Mat frame, gray;

    // GimbalPitch Estimation
    double GimbalPitch = 0.0; //0.45;
    double pitchRate = 0.0;
    double GimbalRoll = 0.0;
    double rollRate = 0.0;
    clock_t start_time = clock();
    clock_t duration;

    // Tether unit transform 
    double tetherScale = 29.348;

    // Fotokite *fotokite = new Fotokite("192.168.2.100", 8080);
    Fotokite *fotokite = new Fotokite("/dev/cu.usbmodem1");

    while (waitKey(30) != 27) {
        //////////// Visual processing ////////////
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };
        zarray_t *detections = apriltag_detector_detect(td, &im);

        //////////// Sensor Info  ////////////
        // Get QX, QY, QZ, QW, relTetherLength, Elevation, relAzimuth
        // Server
        double QX = fotokite->getQX();
        double QY = fotokite->getQY();
        double QZ = fotokite->getQZ();
        double QW = fotokite->getQW();
        double t3 = +2.0 * (QW * QX + QY * QZ); // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        double t4 = +1.0 - 2.0 * (QX*QX + QY * QY);
        double yaw = atan(t3/t4); // need to find the sign of yaw

        double relTetherLength = fotokite->getRelTetherLength()/tetherScale; // all signs need to be verified 
        double Elevation = 1.57 - fotokite->getElevation();
        double relAzimuth = -fotokite->getRelAzimuth();

        //Test
        // double QX = 1;
        // double QY = 0;
        // double QZ = 0;
        // double QW = 0;
        // double t3 = +2.0 * (QW * QX + QY * QZ); // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // double t4 = +1.0 - 2.0 * (QX*QX + QY * QY);
        // double yaw = atan(t3/t4); // need to find the sign of yaw

        // double relTetherLength = 100/tetherScale;
        // double Elevation = 0.707;
        // double relAzimuth = 0;

        double x = relTetherLength*cos(Elevation)*cos(relAzimuth);
        double y = relTetherLength*sin(Elevation);
        double z = -relTetherLength*cos(Elevation)*sin(relAzimuth); 
        double theta_z = GimbalRoll; //http://planning.cs.uiuc.edu/node102.html 
        double theta_y = yaw;
        double theta_x = GimbalPitch;

        for (int i = 0; i < zarray_size(detections); i++) { // could make this for loop only iterate onece
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

            matd_t *pose = homography_to_pose(det->H, fx, fy, cx, cy); // get camera pose from homography, Tag center is (0,0,0)

            matd_t *intrinsic = matd_create(3,4); // intrinsic matrix
            MATD_EL(intrinsic, 0, 0) = fx;
            MATD_EL(intrinsic, 0, 1) = 0;
            MATD_EL(intrinsic, 0, 2) = cx;
            MATD_EL(intrinsic, 0, 3) = 0;
            MATD_EL(intrinsic, 1, 0) = 0;
            MATD_EL(intrinsic, 1, 1) = fy;
            MATD_EL(intrinsic, 1, 2) = cy;
            MATD_EL(intrinsic, 1, 3) = 0;
            MATD_EL(intrinsic, 2, 0) = 0;
            MATD_EL(intrinsic, 2, 1) = 0;
            MATD_EL(intrinsic, 2, 2) = 1;
            MATD_EL(intrinsic, 2, 3) = 0;

            matd_t *homo = matd_multiply(intrinsic, pose); // get 3x4 homography instead of the 3x3 in the library

            matd_t *boundary_points = matd_create(4,4); // 4 boundary points for the box, tag is the bottom side
            MATD_EL(boundary_points, 0, 0) = -1;
            MATD_EL(boundary_points, 0, 1) = -1;
            MATD_EL(boundary_points, 0, 2) = 1;
            MATD_EL(boundary_points, 0, 3) = 1;
            MATD_EL(boundary_points, 1, 0) = -1;
            MATD_EL(boundary_points, 1, 1) = 1;
            MATD_EL(boundary_points, 1, 2) = 1;
            MATD_EL(boundary_points, 1, 3) = -1;
            MATD_EL(boundary_points, 2, 0) = 1;
            MATD_EL(boundary_points, 2, 1) = 1;
            MATD_EL(boundary_points, 2, 2) = 1;
            MATD_EL(boundary_points, 2, 3) = 1;
            MATD_EL(boundary_points, 3, 0) = 1;
            MATD_EL(boundary_points, 3, 1) = 1;
            MATD_EL(boundary_points, 3, 2) = 1;
            MATD_EL(boundary_points, 3, 3) = 1;

            matd_t *end_points = matd_multiply(homo,boundary_points); // transform the box into camera coordinates and connecting lines
            line(frame, Point(MATD_EL(end_points,0,0)/MATD_EL(end_points,2,0), MATD_EL(end_points,1,0)/MATD_EL(end_points,2,0)),
                     Point(MATD_EL(end_points,0,1)/MATD_EL(end_points,2,1), MATD_EL(end_points,1,1)/MATD_EL(end_points,2,1)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(MATD_EL(end_points,0,1)/MATD_EL(end_points,2,1), MATD_EL(end_points,1,1)/MATD_EL(end_points,2,1)),
                     Point(MATD_EL(end_points,0,2)/MATD_EL(end_points,2,2), MATD_EL(end_points,1,2)/MATD_EL(end_points,2,2)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(MATD_EL(end_points,0,2)/MATD_EL(end_points,2,2), MATD_EL(end_points,1,2)/MATD_EL(end_points,2,2)),
                     Point(MATD_EL(end_points,0,3)/MATD_EL(end_points,2,3), MATD_EL(end_points,1,3)/MATD_EL(end_points,2,3)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(MATD_EL(end_points,0,3)/MATD_EL(end_points,2,3), MATD_EL(end_points,1,3)/MATD_EL(end_points,2,3)),
                     Point(MATD_EL(end_points,0,0)/MATD_EL(end_points,2,0), MATD_EL(end_points,1,0)/MATD_EL(end_points,2,0)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(Point(det->p[0][0], det->p[0][1])),
                     Point(MATD_EL(end_points,0,1)/MATD_EL(end_points,2,1), MATD_EL(end_points,1,1)/MATD_EL(end_points,2,1)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(Point(det->p[1][0], det->p[1][1])),
                     Point(MATD_EL(end_points,0,2)/MATD_EL(end_points,2,2), MATD_EL(end_points,1,2)/MATD_EL(end_points,2,2)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(Point(det->p[2][0], det->p[2][1])),
                     Point(MATD_EL(end_points,0,3)/MATD_EL(end_points,2,3), MATD_EL(end_points,1,3)/MATD_EL(end_points,2,3)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(Point(det->p[3][0], det->p[3][1])),
                     Point(MATD_EL(end_points,0,0)/MATD_EL(end_points,2,0), MATD_EL(end_points,1,0)/MATD_EL(end_points,2,0)),
                     Scalar(0, 0xff, 0), 2);

        //////////// Controls  ////////////
            // get homogeneous transformation from tag to fotokite: inverse? of (camera) pose
            matd_t *g_ft_measured = pose; // http://answers.unity3d.com/storage/temp/12048-lefthandedtorighthanded.pdf
            
            MATD_EL(g_ft_measured, 0, 2) = -MATD_EL(g_ft_measured, 0, 2); // change to conventional way, here comes the controls
            MATD_EL(g_ft_measured, 1, 2) = -MATD_EL(g_ft_measured, 1, 2);
            MATD_EL(g_ft_measured, 2, 0) = -MATD_EL(g_ft_measured, 2, 0);
            MATD_EL(g_ft_measured, 2, 1) = -MATD_EL(g_ft_measured, 2, 1); // rotation change
            MATD_EL(g_ft_measured, 2, 3) = -MATD_EL(g_ft_measured, 2, 3); // translation z change

            // get desired homogeneous transformation from tag to fotokite
            // based on the pose from putting the tag at a desired position and orientation in front of the camera
            // inverse the pose
            matd_t *g_ft_desired = matd_create(4,4); 
            MATD_EL(g_ft_desired, 0, 0) = 0; // from the screen output
            MATD_EL(g_ft_desired, 0, 1) = 1;
            MATD_EL(g_ft_desired, 0, 2) = 0;
            MATD_EL(g_ft_desired, 0, 3) = 0;
            MATD_EL(g_ft_desired, 1, 0) = -1;
            MATD_EL(g_ft_desired, 1, 1) = 0;
            MATD_EL(g_ft_desired, 1, 2) = 0;
            MATD_EL(g_ft_desired, 1, 3) = 0;
            MATD_EL(g_ft_desired, 2, 0) = 0;
            MATD_EL(g_ft_desired, 2, 1) = 0;
            MATD_EL(g_ft_desired, 2, 2) = 1;
            MATD_EL(g_ft_desired, 2, 3) = -10;
            MATD_EL(g_ft_desired, 3, 0) = 0;
            MATD_EL(g_ft_desired, 3, 1) = 0;
            MATD_EL(g_ft_desired, 3, 2) = 0;
            MATD_EL(g_ft_desired, 3, 3) = 1;

            // plot the desire tag position
            matd_t *homo_desired = matd_multiply(intrinsic, g_ft_desired); // get 3x4 homography instead of the 3x3 in the library

            matd_t *boundary_points_desired = matd_create(4,4); // 4 boundary points for the box, tag is the bottom side
            MATD_EL(boundary_points_desired, 0, 0) = -1;
            MATD_EL(boundary_points_desired, 0, 1) = -1;
            MATD_EL(boundary_points_desired, 0, 2) = 1;
            MATD_EL(boundary_points_desired, 0, 3) = 1;
            MATD_EL(boundary_points_desired, 1, 0) = -1;
            MATD_EL(boundary_points_desired, 1, 1) = 1;
            MATD_EL(boundary_points_desired, 1, 2) = 1;
            MATD_EL(boundary_points_desired, 1, 3) = -1;
            MATD_EL(boundary_points_desired, 2, 0) = 0;
            MATD_EL(boundary_points_desired, 2, 1) = 0;
            MATD_EL(boundary_points_desired, 2, 2) = 0;
            MATD_EL(boundary_points_desired, 2, 3) = 0;
            MATD_EL(boundary_points_desired, 3, 0) = 1;
            MATD_EL(boundary_points_desired, 3, 1) = 1;
            MATD_EL(boundary_points_desired, 3, 2) = 1;
            MATD_EL(boundary_points_desired, 3, 3) = 1;

            matd_t *end_points_desired = matd_multiply(homo_desired,boundary_points_desired); // transform the box into camera coordinates and connecting lines
            line(frame, Point(MATD_EL(end_points_desired,0,0)/MATD_EL(end_points_desired,2,0), MATD_EL(end_points_desired,1,0)/MATD_EL(end_points_desired,2,0)),
                     Point(MATD_EL(end_points_desired,0,1)/MATD_EL(end_points_desired,2,1), MATD_EL(end_points_desired,1,1)/MATD_EL(end_points_desired,2,1)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(MATD_EL(end_points_desired,0,1)/MATD_EL(end_points_desired,2,1), MATD_EL(end_points_desired,1,1)/MATD_EL(end_points_desired,2,1)),
                     Point(MATD_EL(end_points_desired,0,2)/MATD_EL(end_points_desired,2,2), MATD_EL(end_points_desired,1,2)/MATD_EL(end_points_desired,2,2)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(MATD_EL(end_points_desired,0,2)/MATD_EL(end_points_desired,2,2), MATD_EL(end_points_desired,1,2)/MATD_EL(end_points_desired,2,2)),
                     Point(MATD_EL(end_points_desired,0,3)/MATD_EL(end_points_desired,2,3), MATD_EL(end_points_desired,1,3)/MATD_EL(end_points_desired,2,3)),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(MATD_EL(end_points_desired,0,3)/MATD_EL(end_points_desired,2,3), MATD_EL(end_points_desired,1,3)/MATD_EL(end_points_desired,2,3)),
                     Point(MATD_EL(end_points_desired,0,0)/MATD_EL(end_points_desired,2,0), MATD_EL(end_points_desired,1,0)/MATD_EL(end_points_desired,2,0)),
                     Scalar(0, 0xff, 0), 2);


            // done plotting, change g_ft_desired to right handed
            MATD_EL(g_ft_desired, 0, 2) = -MATD_EL(g_ft_desired, 0, 2);
            MATD_EL(g_ft_desired, 1, 2) = -MATD_EL(g_ft_desired, 1, 2);
            MATD_EL(g_ft_desired, 2, 0) = -MATD_EL(g_ft_desired, 2, 0);
            MATD_EL(g_ft_desired, 2, 1) = -MATD_EL(g_ft_desired, 2, 1); // rotation change
            MATD_EL(g_ft_desired, 2, 3) = -MATD_EL(g_ft_desired, 2, 3); // translation z change

            // inverse g_ft_desired
            matd_t *g_ft_desired_inv = matd_inverse(g_ft_desired);

            matd_t *g_gf_measured = matd_create(4,4); //http://nghiaho.com/?page_id=846
            MATD_EL(g_gf_measured, 0, 0) = cos(theta_z)*cos(theta_y);
            MATD_EL(g_gf_measured, 0, 1) = cos(theta_z)*sin(theta_y)*sin(theta_x)-sin(theta_z)*cos(theta_x);
            MATD_EL(g_gf_measured, 0, 2) = cos(theta_z)*sin(theta_y)*cos(theta_x)+sin(theta_z)*sin(theta_x);
            MATD_EL(g_gf_measured, 0, 3) = x;
            MATD_EL(g_gf_measured, 1, 0) = sin(theta_z)*cos(theta_y);
            MATD_EL(g_gf_measured, 1, 1) = sin(theta_z)*sin(theta_y)*sin(theta_x)+cos(theta_z)*cos(theta_x);
            MATD_EL(g_gf_measured, 1, 2) = sin(theta_z)*sin(theta_y)*cos(theta_x)-cos(theta_z)*sin(theta_x);
            MATD_EL(g_gf_measured, 1, 3) = y;
            MATD_EL(g_gf_measured, 2, 0) = -1*sin(theta_y);
            MATD_EL(g_gf_measured, 2, 1) = cos(theta_y)*sin(theta_x);
            MATD_EL(g_gf_measured, 2, 2) = cos(theta_y)*cos(theta_x);
            MATD_EL(g_gf_measured, 2, 3) = z;
            MATD_EL(g_gf_measured, 3, 0) = 0;
            MATD_EL(g_gf_measured, 3, 1) = 0;
            MATD_EL(g_gf_measured, 3, 2) = 0;
            MATD_EL(g_gf_measured, 3, 3) = 1;
            // that finishes compute g_gf_measured

            matd_t *temp = matd_multiply(g_gf_measured, g_ft_measured);
            matd_t *g_gf_controlled = matd_multiply(temp, g_ft_desired_inv);
            // rot2euler(g_gf_controlled);

            double x_controlled = MATD_EL(g_gf_controlled, 0, 3);
            double y_controlled = MATD_EL(g_gf_controlled, 1, 3);
            double z_controlled = MATD_EL(g_gf_controlled, 2, 3);

            // here solves the inverse kinematics 
            double r32 = MATD_EL(g_gf_controlled, 2, 1);
            double r33 = MATD_EL(g_gf_controlled, 2, 2);
            double relTetherLength_controlled = sqrt(x_controlled*x_controlled+y_controlled*y_controlled+z_controlled*z_controlled);
            double Elevation_controlled = asin(y_controlled/relTetherLength_controlled);
            double relAzimuth_controlled = -atan2(z_controlled, x_controlled); 
            if(abs(relAzimuth - relAzimuth_controlled)>3.14){
                relAzimuth_controlled = - relAzimuth_controlled;
            }
            // http://nghiaho.com/?page_id=846
            double yaw_controlled = atan2(-MATD_EL(g_gf_controlled, 2, 0),sqrt(r32*r32+r33*r33)); // theta_y
            double GimbalPitch_controlled = atan2(MATD_EL(g_gf_controlled, 2, 1),MATD_EL(g_gf_controlled, 2, 2)); // theta_x
            // double GimbalRoll_controlled = atan2(MATD_EL(g_gf_controlled, 1, 0),MATD_EL(g_gf_controlled, 0, 0)); // theta_z

            // here goes the actual commands to FK
            // tether control
            double tether_tolerance = 1; // need to see the actual encoder value
            if (relTetherLength_controlled < relTetherLength - tether_tolerance){
            	fotokite->posL(-5);// decrease tether length
                // cout<<"tether: "<<"-10"<<"\t";
            }
            else if (relTetherLength_controlled > relTetherLength + tether_tolerance){
            	fotokite->posL(+5);// increase tether length
                // cout<<"tether: "<<"+10"<<"\t";
            }
            else {
                fotokite->posL(0);
                // cout<<"tether: "<<"  "<<"\t";
            }

            // elevation control
            double Elevation_tolerance = 0.2; // need to see the actual value
            if (Elevation_controlled < Elevation - Elevation_tolerance){
            	fotokite->posV(-0.1);// decrease elevation
                // cout<<"elevation: "<<"-0.1"<<"\t";
            }
            else if (Elevation_controlled > Elevation + Elevation_tolerance){
            	fotokite->posV(+0.1);// increase elevation
                // cout<<"elevation: "<<"+0.1"<<"\t";
            }
            else {
                fotokite->posV(0);
                // cout<<"elevation: "<<"  "<<"\t";
            }

            // azimuth control
            double relAzimuth_tolerance = 0.2; // need to see the actual value
            if (relAzimuth_controlled < relAzimuth - relAzimuth_tolerance){
            	fotokite->posH(+0.1);// decrease relAzimuth
                // cout<<"azimuth: "<<"-0.1"<<"\t";
            }
            else if (relAzimuth_controlled > relAzimuth + relAzimuth_tolerance){
            	fotokite->posH(-0.1);// increase relAzimuth
                // cout<<"azimuth: "<<"+0.1"<<"\t";
            }
            else {
                fotokite->posH(0);
                // cout<<"azimuth: "<<"  "<<"\t";
            }

            // camera yaw control
            double yaw_tolerance = 0.2;
            if (yaw_controlled < yaw - yaw_tolerance){
                fotokite->yaw(-0.1);// decrease yaw
                // cout<<"yaw:    "<<"-0.1"<<"\t";
            }
            else if (yaw_controlled > yaw + yaw_tolerance){
                fotokite->yaw(+0.2);// increase yaw
                // cout<<"yaw:  "<<"+0.1"<<"\t";
            }
            else {
                fotokite->yaw(0);
                // cout<<"yaw   "<<"  "<<"\t";
            }

            // camera pitch and roll control (first estimate, then update)
            duration = (clock()-start_time); // ms to s 
            double duraton_in_s = float(duration)/CLOCKS_PER_SEC;
            // cout<<duration<<endl;
            GimbalPitch = GimbalPitch + pitchRate * duraton_in_s;
            // cout<<"pitchRate: "<<pitchRate<<endl;
            // cout<<"duration_in_s: "<<duraton_in_s<<endl;
            // cout<<"GimbalPitch: "<<GimbalPitch<<endl;
            GimbalRoll = GimbalRoll + rollRate * duraton_in_s; 

            // update GimbalPitch here 
            double GimbalPitch_tolerance = 0.2;
            if (GimbalPitch_controlled < GimbalPitch - GimbalPitch_tolerance){
                pitchRate = -0.2; 
                fotokite->gimbalPitch(pitchRate);// decrease pitch
                // cout<<"pitch: "<<"-0.1"<<"\t";
            }
            else if (GimbalPitch_controlled > GimbalPitch + GimbalPitch_tolerance){
                pitchRate = +0.2; 
                fotokite->gimbalPitch(pitchRate);// increase pitch
                // cout<<"pitch: "<<"+0.1"<<"\t";
            }
            else {
                pitchRate = 0; 
                fotokite->gimbalPitch(pitchRate);
                // cout<<"pitch: "<<"  "<<"\t";
            }

            // // update GimbalRoll here
            // double GimbalRoll_tolerance = 0.2;
            // if (GimbalRoll_controlled < GimbalRoll - GimbalRoll_tolerance){
            //     // fotokite->gimbalRoll(-0.1);// decrease roll
            //     // cout<<"roll: "<<"-0.1"<<endl;
            // }
            // else if (GimbalRoll_controlled > GimbalRoll + GimbalRoll_tolerance){
            //     // fotokite->gimbalRoll(+0.1);// increase roll
            //     // cout<<"roll: "<<"+0.1"<<endl;
            // }
            // else {
            //     // cout<<"roll: "<<"  "<<endl;
            // }
            cout<<"x_s: "<<x<<", y_s: "<<y<<", z_s: "<<z<<endl;
            cout<<"x_c: "<<x_controlled<<", y_c: "<<y_controlled<<", z_c: "<<z_controlled<<endl;
            cout<<"Tether_s: "<<relTetherLength<<", Elevation_s: "<<Elevation/3.1415926*180<<", Azimuth_s: "<<relAzimuth/3.1415926*180<<", Yaw_s: "<<yaw/3.1415926*180<<", Pitch_s: "<<GimbalPitch/3.1415926*180<<endl;
            cout<<"Tether_c: "<<relTetherLength_controlled<<", Elevation_c: "<<Elevation_controlled/3.1415926*180<<", Azimuth_c: "<<relAzimuth_controlled/3.1415926*180<<", Yaw_c: "<<yaw_controlled/3.1415926*180<<", Pitch_c: "<<GimbalPitch_controlled/3.1415926*180<<endl;
            start_time = clock();

        }
        if(zarray_size(detections)==0){
            cout<<"x_s: "<<x<<", y_s: "<<y<<", z_s: "<<z<<endl;
            cout<<"Tether_s: "<<relTetherLength<<", Elevation_s: "<<Elevation/3.1415926*180<<", Azimuth_s: "<<relAzimuth/3.1415926*180<<", Yaw_s: "<<yaw/3.1415926*180<<", Pitch_s: "<<GimbalPitch/3.1415926*180<<endl;

            duration = (clock()-start_time); // ms to s 
            double duraton_in_s = float(duration)/CLOCKS_PER_SEC;
            // cout<<duration<<endl;
            GimbalPitch = GimbalPitch + pitchRate * duraton_in_s;
            GimbalRoll = GimbalRoll + rollRate * duraton_in_s; 

            fotokite->posL(0);
            fotokite->posV(0);
            fotokite->posH(0);
            fotokite->yaw(0);
            fotokite->gimbalPitch(0);
            pitchRate = 0; 
            start_time = clock();

        }
        
        zarray_destroy(detections);

        imshow("Tag Detections", frame);

    }

    apriltag_detector_destroy(td);
    if (!strcmp(famname, "tag36h11"))
        tag36h11_destroy(tf);
    else if (!strcmp(famname, "tag36h10"))
        tag36h10_destroy(tf);
    else if (!strcmp(famname, "tag36artoolkit"))
        tag36artoolkit_destroy(tf);
    else if (!strcmp(famname, "tag25h9"))
        tag25h9_destroy(tf);
    else if (!strcmp(famname, "tag25h7"))
        tag25h7_destroy(tf);
    getopt_destroy(getopt);

    delete fotokite;
    return 0;
}
