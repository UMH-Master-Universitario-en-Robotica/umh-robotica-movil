/******************************************************************************/
/*****                            callbacks.cpp                           *****/
/*****                             Raul Tapia                             *****/
/*****                           Robótica Móvil                           *****/
/*****   Master Universitario en Robótica - Universidad Miguel Hernández  *****/
/******************************************************************************/

/**
 * @file    callbacks.cpp
 * @brief   Callbacks for stereo camera and ground-truth measures.
 * @author  Raul Tapia
 */

#include "visual_odometry/visual_odometry.h"

/*** --- Callback for left and rigth images --- ***/
void VisualOdometry::syncCallback(){
        /*** --- Compute detectors and descriptors --- ***/
        getDetectorsAndDescriptors(currentImage0, currentKeypoints0, currentDescriptors0, "left");

        /*** --- Compute transform between previous and current image --- ***/
        if(!firstStereoImage) {
                getMatches(prevImage0, prevKeypoints0, prevDescriptors0, prevPosition2D0, \
                           currentImage0, currentKeypoints0, currentDescriptors0, currentPosition2D0, "prev-current");

                sortCoincidences(prevPosition2D0, currentPosition2D0, prevPosition3D, prevPosition2DWith3D);

                /*** --- Synthetic data for testing --- ***/
                // prevPosition3D.clear();
                // currentPosition2D0.clear();
                // for(unsigned int i = 0; i < 30; i++) {
                //         double x = rand()%10;
                //         double y = rand()%10;
                //         double z = rand()%5 + 0.5;
                //         prevPosition3D.push_back(cv::Point3f(x,y,z));
                //         x += 0.1;
                //         currentPosition2D0.push_back(cv::Point2f(FX_0*x/z + CX_0, FY_0*y/z + CY_0));
                // }

                if(prevPosition3D.size() > 10) {
                        TransMatrix t = getAlignment(prevPosition3D, currentPosition2D0);
                        updateRobotPose(t);
                }
        }
        else if(initTime > 0) {
                firstStereoImage = false;
        }

        /*** --- Compute 3D position of corresponding pairs of points for next step --- ***/
        getDetectorsAndDescriptors(currentImage1, currentKeypoints1, currentDescriptors1, "rigth");

        getMatches(currentImage0, currentKeypoints0, currentDescriptors0, currentPosition2D0, \
                   currentImage1, currentKeypoints1, currentDescriptors1, currentPosition2D1, "left-rigth");

        get3DPointFromStereo(currentPosition2D0, currentPosition2D1, prevPosition3D);
        prevPosition2DWith3D = currentPosition2D0;

        /*** --- Prepare next step --- ***/
        currentImage0.copyTo(prevImage0);
        prevKeypoints0 = currentKeypoints0;
        currentDescriptors0.copyTo(prevDescriptors0);
}

/*** --- Callback for left image --- ***/
void VisualOdometry::cam0Callback(const sensor_msgs::Image::ConstPtr& msg){
        /*** --- Get image --- ***/
        cv_bridge::CvImagePtr p = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvtColor(p->image, currentImage0, CV_BGR2GRAY);

        /*** --- Write time before syncCallback() --- ***/
        lastTime = msg->header.stamp;

        /*** --- Call syncCallback() --- ***/
        if(flagSync) {
                syncCallback();
                flagSync = false;
        }
        else{
                flagSync = true;
        }
}

/*** --- Callback for rigth image --- ***/
void VisualOdometry::cam1Callback(const sensor_msgs::Image::ConstPtr& msg){
        /*** --- Get image --- ***/
        cv_bridge::CvImagePtr p = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvtColor(p->image, currentImage1, CV_BGR2GRAY);

        /*** --- Write time before syncCallback() --- ***/
        lastTime = msg->header.stamp;

        /*** --- Call syncCallback() --- ***/
        if(flagSync) {
                syncCallback();
                flagSync = false;
        }
        else{
                flagSync = true;
        }
}

/*** --- Callback for ground-truth --- ***/
void VisualOdometry::viconCallback(const geometry_msgs::TransformStamped& msg){
        geometry_msgs::PointStamped vicon;

        /*** --- Remove initial position --- ***/
        if(firstGroundTruth.x == 0 && firstGroundTruth.x == 0 && firstGroundTruth.x == 0) {
                firstGroundTruth.x = msg.transform.translation.x;
                firstGroundTruth.y = msg.transform.translation.y;
                firstGroundTruth.z = msg.transform.translation.z;
        }

        /*** --- Set msg --- ***/
        vicon.header = msg.header;
        vicon.header.frame_id = "map";
        vicon.point.x = msg.transform.translation.x - firstGroundTruth.x;
        vicon.point.y = msg.transform.translation.y - firstGroundTruth.y;
        vicon.point.z = msg.transform.translation.z - firstGroundTruth.z;

        /*** --- Initialize time for sync --- ***/
        if(firstStereoImage) {
                initTime = vicon.header.stamp.sec*1.0 + vicon.header.stamp.nsec*1e-9;
        }

        /*** --- Get correction --- ***/
        if(CORRECTION_MODE && contCorrection == 3) {
                robotPositionForCorrection.x = -vicon.point.y + 0.01*(double)(rand()%20-10);
                robotPositionForCorrection.y =  vicon.point.z + 0.01*(double)(rand()%20-10);
                robotPositionForCorrection.z =  vicon.point.x + 0.01*(double)(rand()%20-10);
                flagCorrection = true;
                contCorrection = 0;
        }
        contCorrection++;

        /*** --- Log data --- ***/
        groundTruthFile << (vicon.header.stamp.sec*1.0 + vicon.header.stamp.nsec*1e-9) - initTime << " ";
        groundTruthFile << vicon.point.x << " " << vicon.point.y << " " << vicon.point.z << std::endl;

        /*** --- Publish data --- ***/
        groundTruthPublisher.publish(vicon);

}

/*** --- Callback for ground-truth --- ***/
void VisualOdometry::leicaCallback(const geometry_msgs::PointStamped& msg){
        geometry_msgs::PointStamped leica;

        /*** --- Remove initial position --- ***/
        if(firstGroundTruth.x == 0 && firstGroundTruth.x == 0 && firstGroundTruth.x == 0) {
                firstGroundTruth.x = msg.point.x;
                firstGroundTruth.y = msg.point.y;
                firstGroundTruth.z = msg.point.z;
        }

        /*** --- Set msg --- ***/
        leica.header = msg.header;
        leica.header.frame_id = "map";
        leica.point.x = msg.point.x - firstGroundTruth.x;
        leica.point.y = msg.point.y - firstGroundTruth.y;
        leica.point.z = msg.point.z - firstGroundTruth.z;

        /*** --- Initialize time for sync --- ***/
        if(firstStereoImage) {
                initTime = leica.header.stamp.sec*1.0 + leica.header.stamp.nsec*1e-9;
        }

        /*** --- Get correction --- ***/
        if(CORRECTION_MODE && contCorrection == 3) {
                robotPositionForCorrection.x = -leica.point.y + 0.01*(double)(rand()%20-10);
                robotPositionForCorrection.y =  leica.point.z + 0.01*(double)(rand()%20-10);
                robotPositionForCorrection.z =  leica.point.x + 0.01*(double)(rand()%20-10);
                flagCorrection = true;
                contCorrection = 0;
        }
        contCorrection++;

        /*** --- Log data --- ***/
        groundTruthFile << (leica.header.stamp.sec*1.0 + leica.header.stamp.nsec*1e-9) - initTime << " ";
        groundTruthFile << leica.point.x << " " << leica.point.y << " " << leica.point.z << std::endl;

        /*** --- Publish data --- ***/
        groundTruthPublisher.publish(leica);
}
