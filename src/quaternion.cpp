/******************************************************************************/
/*****                           quaternion.cpp                           *****/
/*****                             Raul Tapia                             *****/
/*****                           Robótica Móvil                           *****/
/*****   Master Universitario en Robótica - Universidad Miguel Hernández  *****/
/******************************************************************************/

/**
 * @file    quaternion.cpp
 * @brief   Functions to operate with quaternions.
 * @author  Raul Tapia
 */

#include "visual_odometry/quaternion.h"

/*** --- Convert rotation matrix to quaternion --- ***/
// @author: https://gist.github.com/shubh-agrawal/76754b9bfb0f4143819dbd146d15d4c8
void getQuaternion(cv::Mat R, double Q[]) {
        double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);

        if (trace > 0.0) {
                double s = sqrt(trace + 1.0);
                Q[3] = (s * 0.5);
                s = 0.5 / s;
                Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
                Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
                Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
        }

        else {
                int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0);
                int j = (i + 1) % 3;
                int k = (i + 2) % 3;

                double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
                Q[i] = s * 0.5;
                s = 0.5 / s;

                Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
                Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
                Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
        }
}

/*** --- Muliply two quaterions --- ***/
void multiplyQuaternion(const double* q1,const double* q2, double* q) {
        const float x1 = q1[0];
        const float y1 = q1[1];
        const float z1 = q1[2];
        const float r1 = q1[3];

        const float x2 = q2[0];
        const float y2 = q2[1];
        const float z2 = q2[2];
        const float r2 = q2[3];

        q[0] = x1*r2 + r1*x2 + y1*z2 - z1*y2;
        q[1] = r1*y2 - x1*z2 + y1*r2 + z1*x2;
        q[2] = r1*z2 + x1*y2 - y1*x2 + z1*r2;
        q[3] = r1*r2 - x1*x2 - y1*y2 - z1*z2;
}
