/******************************************************************************/
/*****                          visual_odometry.h                         *****/
/*****                             Raul Tapia                             *****/
/*****                           Robótica Móvil                           *****/
/*****   Master Universitario en Robótica - Universidad Miguel Hernández  *****/
/******************************************************************************/

/**
 * @file    visual_odometry.h
 * @brief   Header file.
 * @author  Raul Tapia
 */

#include <stdlib.h>
#include <iostream>
#include <fstream>

#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*** --- Modes --- ***/
#define DEBUG_MODE false
#define CORRECTION_MODE false
#define ALPHA 0.3

/*** --- Features --- ***/
#define NUM_KEYPOINTS 1000
#define GOOD_MATCHES_FACTOR 0.2

/*** --- Camera calibration --- ***/
#define FX_0 458.654
#define FY_0 457.296
#define CX_0 367.215
#define CY_0 248.375
#define CX_1 379.999
#define CY_1 255.238
#define F0   458.654
#define F1   457.587
#define BASELINE -0.110073808127187

#define FX 458.654
#define FY 457.296
#define CX 367.215
#define CY 248.375
#define K1 -0.28340811
#define K2 0.07395907
#define P1 0.00019359
#define P2 1.76187114e-05

/*** --- Types --- ***/
typedef struct{
    double r[3][3];
    double p[3];
} TransMatrix;

/*** --- Classes --- ***/
class VisualOdometry {
    private:
        double initTime = -1;

        ros::Time lastTime;

        ros::Publisher visualOdometryPublisher;
        ros::Publisher groundTruthPublisher;
        cv::Point3f firstGroundTruth;

        std::ofstream visualOdometryFile;
        std::ofstream groundTruthFile;

        bool flagSync = false;
        bool firstStereoImage = true;
        bool flagCorrection = false;
        unsigned int contCorrection = 0;

        cv::Mat currentImage0;
        cv::Mat currentImage1;
        cv::Mat prevImage0;
        cv::Mat prevImage1;

        std::vector<cv::KeyPoint> currentKeypoints0;
        std::vector<cv::KeyPoint> currentKeypoints1;
        std::vector<cv::KeyPoint> prevKeypoints0;
        cv::Mat currentDescriptors0;
        cv::Mat currentDescriptors1;
        cv::Mat prevDescriptors0;

        std::vector<cv::Point2f> currentPosition2D0;
        std::vector<cv::Point2f> currentPosition2D1;
        std::vector<cv::Point2f> prevPosition2D0;

        std::vector<cv::Point2f> prevPosition2DWith3D;
        std::vector<cv::Point3f> prevPosition3D;

        void syncCallback();
        void getDetectorsAndDescriptors(cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat&, const char*);
        void getMatches(cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat&, std::vector<cv::Point2f>&, \
                        cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat&, std::vector<cv::Point2f>&, const char*);
        void get3DPointFromStereo(std::vector<cv::Point2f>&, std::vector<cv::Point2f>&, std::vector<cv::Point3f>&);
        void sortCoincidences(std::vector<cv::Point2f>&, std::vector<cv::Point2f>&, std::vector<cv::Point3f>&, std::vector<cv::Point2f>&);
        TransMatrix getAlignment(std::vector<cv::Point3f>&, std::vector<cv::Point2f>&);
        void updateRobotPose(TransMatrix);

    public:
        cv::Point3f robotPosition;
        double robotOrientation[4];
        cv::Point3f robotPositionForCorrection;

        VisualOdometry(ros::NodeHandle);
        ~VisualOdometry(void);

        void cam0Callback(const sensor_msgs::Image::ConstPtr&);
        void cam1Callback(const sensor_msgs::Image::ConstPtr&);
        void viconCallback(const geometry_msgs::TransformStamped&);
        void leicaCallback(const geometry_msgs::PointStamped&);

};
