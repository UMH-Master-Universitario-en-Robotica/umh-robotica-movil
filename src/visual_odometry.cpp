/******************************************************************************/
/*****                         visual_odometry.cpp                        *****/
/*****                             Raul Tapia                             *****/
/*****                           Robótica Móvil                           *****/
/*****   Master Universitario en Robótica - Universidad Miguel Hernández  *****/
/******************************************************************************/

/**
 * @file    visual_odometry.cpp
 * @brief   Mobile robot pose estimation using visual odometry.
 * @author  Raul Tapia
 */

#include "visual_odometry/visual_odometry.h"
#include "visual_odometry/epnp.h"
#include "visual_odometry/quaternion.h"

/*** --- Absolute value of a number --- ***/
double getAbs(double x){
        if (x < 0)
                return -x;
        return x;
}

/*** --- Constructor for VisualOdometry --- ***/
VisualOdometry::VisualOdometry(ros::NodeHandle n){
        /*** --- Initialize publishers --- ***/
        visualOdometryPublisher = n.advertise<geometry_msgs::PointStamped>("/visual_odometry/pose", 100);
        groundTruthPublisher = n.advertise<geometry_msgs::PointStamped>("/visual_odometry/ground_truth", 100);

        /*** --- Open log files --- ***/
        visualOdometryFile.open ("visual_odometry_pose.txt");
        groundTruthFile.open ("visual_odometry_ground_truth.txt");

        /*** --- Initialize robot position --- ***/
        robotPosition = cv::Point3f(0,0,0);
        cv::Mat aux = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
        getQuaternion(aux,robotOrientation);

        /*** --- Windows for debug mode --- ***/
        if(DEBUG_MODE) {
                cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
                cv::namedWindow("rigth", cv::WINDOW_AUTOSIZE);
                cv::namedWindow("left-rigth", cv::WINDOW_AUTOSIZE);
                cv::namedWindow("prev-current", cv::WINDOW_AUTOSIZE);
                cv::waitKey(999);
        }

        std::cout << "Visual odometry initialized" << std::endl;
}

/*** --- Destructor for VisualOdometry --- ***/
VisualOdometry::~VisualOdometry(){
        /*** --- Close log files --- ***/
        visualOdometryFile.close();
        groundTruthFile.close();

        /*** --- Close windows --- ***/
        if(DEBUG_MODE) {
                cv::destroyWindow("left");
                cv::destroyWindow("rigth");
                cv::destroyWindow("left-rigth");
                cv::destroyWindow("prev-current");
        }

        std::cout << "\nVisual odometry finished" << std::endl;
}

/*** --- Detect and describe features using ORB --- ***/
void VisualOdometry::getDetectorsAndDescriptors(cv::Mat& img, std::vector<cv::KeyPoint>& kp, cv::Mat& des, const char* windowName){
        /*** --- Compute detectors and descriptors --- ***/
        cv::Ptr<cv::ORB> orb = cv::ORB::create(NUM_KEYPOINTS, 1.2f, 8, 10);
        orb->detectAndCompute(img, cv::Mat(), kp, des);

        /*** --- Debug matches --- ***/
        if(DEBUG_MODE) {
                cv::Mat auxImage;
                cv::drawKeypoints(img, kp, auxImage, cv::Scalar(0,0,255));
                cv::imshow(windowName, auxImage);
                cv::waitKey(9000);
        }
}

/*** --- Feature mathing --- ***/
void VisualOdometry::getMatches(cv::Mat& imgA, std::vector<cv::KeyPoint>& kpA, cv::Mat& desA, std::vector<cv::Point2f>& pA, \
                                cv::Mat& imgB, std::vector<cv::KeyPoint>& kpB, cv::Mat& desB, std::vector<cv::Point2f>& pB, const char* windowName){
        /*** --- Clear vectors --- ***/
        pA.clear();
        pB.clear();

        /*** --- Get matches --- ***/
        std::vector<cv::DMatch> matches;
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        matcher->match(desA, desB, matches, cv::Mat());

        /*** --- Sort --- ***/
        std::sort(matches.begin(), matches.end());

        /*** --- Remove bad matches --- ***/
        const int numGoodMatches = int(double(matches.size()) * GOOD_MATCHES_FACTOR);
        matches.erase(matches.begin()+numGoodMatches, matches.end());

        /*** --- Debug matches --- ***/
        if(DEBUG_MODE) {
                cv::Mat auxImage;
                cv::drawMatches(imgA, kpA, imgB, kpB, matches, auxImage, cv::Scalar::all(-1));
                cv::imshow(windowName, auxImage);
                cv::waitKey(9000);
        }

        /*** --- Get localization --- ***/
        for(size_t i = 0; i < matches.size(); i++) {
                pA.push_back(kpA[matches[i].queryIdx].pt);
                pB.push_back(kpB[matches[i].trainIdx].pt);
        }
}

/*** --- Compute 3D position from stereo --- ***/
void VisualOdometry::get3DPointFromStereo(std::vector<cv::Point2f>& pLeft, std::vector<cv::Point2f>& pRigth, std::vector<cv::Point3f>& p3D){
        /*** --- Clear vector --- ***/
        p3D.clear();

        /*** --- Compute (x,y,z) using (xL, yL) and (xR, yR) --- ***/
        for(size_t i = 0; i < pLeft.size(); i++) {
                float z = -BASELINE/((pLeft[i].x-CX_0)/F0 - (pRigth[i].x-CX_1)/F1);
                float x = z*((pLeft[i].x-CX_0)/FX_0);
                float y = z*((pLeft[i].y-CY_0)/FY_0);

                cv::Point3f p(x,y,z);
                p3D.push_back(p);
        }
}

/*** --- Match 3D and 2D points --- ***/
void VisualOdometry::sortCoincidences(std::vector<cv::Point2f>& p, std::vector<cv::Point2f>& q, std::vector<cv::Point3f>& p3D, std::vector<cv::Point2f>& pw3D){
        std::vector<cv::Point2f> pSorted2D;
        std::vector<cv::Point3f> pSorted3D;

        /*** --- Sort --- ***/
        for(size_t i = 0; i < p.size(); i++) {
                for(size_t j = 0; j < pw3D.size(); j++) {
                        if(p3D[j].z < 20 && p3D[j].z > 0.5 && getAbs(p[i].x - pw3D[j].x) < 3 && getAbs(p[i].y - pw3D[j].y) < 3) {
                                pSorted2D.push_back(q[i]);
                                pSorted3D.push_back(p3D[j]);
                                break;
                        }
                }
        }

        /*** --- Return by reference --- ***/
        p3D = pSorted3D;
        q = pSorted2D;
}

/*** --- Compute best transform matrix using EPnP --- ***/
TransMatrix VisualOdometry::getAlignment(std::vector<cv::Point3f>& p, std::vector<cv::Point2f>& q){
        /*** --- Instance and setup epnp --- ***/
        epnp PnP;
        PnP.set_internal_parameters(CX, CY, FX, FY);
        PnP.set_maximum_number_of_correspondences(p.size());
        PnP.reset_correspondences();

        /*** --- Add points --- ***/
        for(size_t i = 0; i < p.size(); i++) {
                PnP.add_correspondence(p[i].x, p[i].y, p[i].z, q[i].x, q[i].y);
        }

        /*** --- Compute rotation matrix and translation matrix --- ***/
        TransMatrix result;
        PnP.compute_pose(result.r, result.p);

        return result;
}

/*** --- Update robot position and orientation --- ***/
void VisualOdometry::updateRobotPose(TransMatrix t) {
        if(getAbs(t.p[0]) < 0.1 && getAbs(t.p[1]) < 0.1 && getAbs(t.p[2]) < 0.1) {
                /*** --- Update position --- ***/
                cv::Point3f newRobotPosition;

                newRobotPosition.x = t.r[0][0]*robotPosition.x + \
                                     t.r[0][1]*robotPosition.y + \
                                     t.r[0][2]*robotPosition.z + t.p[0];

                newRobotPosition.y = t.r[1][0]*robotPosition.x + \
                                     t.r[1][1]*robotPosition.y + \
                                     t.r[1][2]*robotPosition.z + t.p[1];

                newRobotPosition.z = t.r[2][0]*robotPosition.x + \
                                     t.r[2][1]*robotPosition.y + \
                                     t.r[2][2]*robotPosition.z + t.p[2];

                if(CORRECTION_MODE && flagCorrection) {
                        robotPosition.x = ALPHA * newRobotPosition.x + (1-ALPHA) * robotPositionForCorrection.x;
                        robotPosition.y = ALPHA * newRobotPosition.y + (1-ALPHA) * robotPositionForCorrection.y;
                        robotPosition.z = ALPHA * newRobotPosition.z + (1-ALPHA) * robotPositionForCorrection.z;
                        flagCorrection = false;
                }
                else{
                        robotPosition.x = newRobotPosition.x;
                        robotPosition.y = newRobotPosition.y;
                        robotPosition.z = newRobotPosition.z;
                }

                /*** --- Update orientation --- ***/
                cv::Mat R = (cv::Mat_<double>(3,3) << t.r[0][0], t.r[0][1], t.r[0][2], \
                             t.r[1][0], t.r[1][1], t.r[1][2], \
                             t.r[2][0], t.r[2][1], t.r[2][2]);

                double q[4];
                getQuaternion(R, q);
                multiplyQuaternion(robotOrientation, q, robotOrientation);
        }

        /*** --- Set msg --- ***/
        geometry_msgs::PointStamped msg;
        msg.header.stamp = lastTime;
        msg.header.frame_id = "map";
        msg.point.x =  robotPosition.z;
        msg.point.y = -robotPosition.x;
        msg.point.z =  robotPosition.y;

        /*** --- Log data --- ***/
        visualOdometryFile << (msg.header.stamp.sec*1.0 + msg.header.stamp.nsec*1e-9) - initTime << " ";
        visualOdometryFile << msg.point.x << " " << msg.point.y << " " << msg.point.z << std::endl;

        /*** --- Publish data --- ***/
        visualOdometryPublisher.publish(msg);
}

/*** --- Main function --- ***/
int main(int argc, char **argv){
        /*** --- ROS setup --- ***/
        ros::init(argc, argv, "visual_odometry");
        ros::NodeHandle n("~");

        /*** --- VisualOdometry instance --- ***/
        VisualOdometry vo(n);

        /*** --- Subscribers --- ***/
        ros::Subscriber sub0 = n.subscribe("/cam0/image_raw", 100, &VisualOdometry::cam0Callback, &vo);
        ros::Subscriber sub1 = n.subscribe("/cam1/image_raw", 100, &VisualOdometry::cam1Callback, &vo);
        ros::Subscriber sub2 = n.subscribe("/vicon/firefly_sbx/firefly_sbx", 100, &VisualOdometry::viconCallback, &vo);
        ros::Subscriber sub3 = n.subscribe("/leica/position", 100, &VisualOdometry::leicaCallback, &vo);

        /*** --- Waiting for callbacks --- ***/
        ros::spin();

        /*** --- That's all folks --- ***/
        return 0;
}
