/*
 * vision.h
 *
 *  Created on: Jan 15, 2016
 *      Author: tigertronics
 */

#ifndef SRC_VISION_VISION_H_
#define SRC_VISION_VISION_H_

#define PI 3.14159265

#include <opencv2/opencv.hpp>
#include <cmath>
#include <ctime>

void visionTest();
void processImage();
double normalize360(double angle);
int getTargetX();
int getTargetY();
int getDistanceToCenter();
float getDistanceY();
//mutex visionMutex;
//	constants for the color rbg values
const Scalar RED = Scalar(0, 0, 255),
BLUE = Scalar(255, 0, 0),
GREEN = Scalar(0, 255, 0),
BLACK = Scalar(0,0,0),
YELLOW = Scalar(0, 255, 255),
//	these are the threshold values in order
LOWER_BOUNDS = Scalar(188,238,188),
UPPER_BOUNDS = Scalar(255,255,255);
//rgb bgr
//lower: 218,241,227 - 227,241,218
//upper: 255,255,255 - 255,255,255

double distance = 0;


//	the size for resing the image
const Size resize = Size(320,240);
const Point centerOfCam = Point(160, 120);
Point center;

//	ignore these
 VideoCapture videoCapture;
 Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy, matGray, matResize, testingMat;
 //Image* myImaqImage;

//	Constants for known variables
//	the height to the top of the target in first stronghold is 97 inches
 const int TOP_TARGET_HEIGHT = 97;
//	the physical height of the camera lens
 const int TOP_CAMERA_HEIGHT = 19;

//	camera details, can usually be found on the datasheets of the camera
 const double VERTICAL_FOV  = 42;
 const double HORIZONTAL_FOV  = 56;
 const double CAMERA_ANGLE = 30;

 bool shouldRun = true;

 int biggestArea = 0;
 int biggestAreaIndex = 0;

 bool buttonPressed = false;

 std::vector<std::vector<cv::Point>> contours;
 std::vector<std::vector<cv::Point>> selected;

 int pictureTaker = 0;

#endif /* SRC_VISION_VISION_H_ */