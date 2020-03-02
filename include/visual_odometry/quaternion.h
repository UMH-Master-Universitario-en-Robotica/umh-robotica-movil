/******************************************************************************/
/*****                             quaterion.h                            *****/
/*****                             Raul Tapia                             *****/
/*****                           Robótica Móvil                           *****/
/*****   Master Universitario en Robótica - Universidad Miguel Hernández  *****/
/******************************************************************************/

/**
 * @file    quaterion.h
 * @brief   Header file.
 * @author  Raul Tapia
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void getQuaternion(cv::Mat, double*);
void multiplyQuaternion(const double*,const double*, double*);
