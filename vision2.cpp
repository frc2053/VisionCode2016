#define PI 3.14159265
#include <cmath>
#include "stdint.h"
#include <ctime>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "networktables/NetworkTable.h"

using namespace std;
using namespace cv;

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
LOWER_BOUNDS = Scalar(62, 0, 71),
UPPER_BOUNDS = Scalar(72,255,255);
//HSV VALUES 
//LIMITS: (0, 0, 0)
//        (180, 255, 255)

double visionDistance = 0;
int imageWrite = 0;

//	the size for resing the image
const Size resizeSize = Size(320,240);
const Point centerOfCam = Point(160, 120);
Point center;

//	ignore these
 VideoCapture videoCapture;
 std::shared_ptr<NetworkTable> visionTable;
 Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy, matGray, matResize, testingMat, matNEWRGB;
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

 vector<vector<Point>> contours;
 vector<vector<Point>> selected;

 int pictureTaker = 0;

/**
 * @param angle a nonnormalized angle
 */
 double normalize360(double angle){
	while(angle >= 360.0)
	{
		angle -= 360.0;
	}
	while(angle < 0.0)
	{
		angle += 360.0;
	}
	return angle;
}

/**
  *
  * reads an image from a live image capture and outputs information to the SmartDashboard or a file
  */
 void processImage(){
	 //videoCapture.set(CV_CAP_PROP_EXPOSURE, 1);
	//buttonPressed = Robot::oi->getDriveJoystick()->GetRawButton(5);
	//printf("IM AM IN PROCESS IMAGE!\n");
 	double x,y,targetX,targetY,azimuth;
 	//int FrameCount = 0;
 	while(true) {
		int64_t t0 = getTickCount();
 		//cout << "frameCount: " << FrameCount << endl;
 		contours.clear();
 		selected.clear();
 //			capture from the axis camera
 		//printf("before read\n");
 		videoCapture.read(matOriginal);
 		//printf("after read\n");
 //			captures from a static file for testing
		if(imageWrite == 0) {
			imwrite("/home/pi/original.jpg", matOriginal);
		}
 		//imwrite("/home/lvuser/original.jpg", matOriginal);
 		resize(matOriginal, matResize, resizeSize, 0, 0, INTER_LINEAR);
		cvtColor(matResize, matHSV, COLOR_BGR2HSV);
		if(imageWrite == 0) {
			imwrite("/home/pi/hsv.jpg", matHSV);
		}
		//cvtColor(matHSV, matNEWRGB, COLOR_HSV2RGB);
		//if(imageWrite == 0) {
		//	imwrite("/home/pi/newrgb.jpg", matNEWRGB);
		//}
 		inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
		if(imageWrite == 0) {
			imwrite("/home/pi/thresh.jpg", matThresh);
		}
		imageWrite++;
 		findContours(matThresh, contours, matHeirarchy, RETR_EXTERNAL,
 				CHAIN_APPROX_SIMPLE);
 		//cout << "Size of contours: " << contours.size() << endl;
 //			make sure the contours that are detected are at least 20x20
 //			pixels with an area of 400 and an aspect ration greater then 1
 		//printf("looping\n");
 		for (unsigned int i = 0; i < contours.size();  i++) {
 			Rect rec = boundingRect(contours[i]);
			float aspect = (float)rec.width/(float)rec.height;
			if(aspect > 1.0 && rec.area() > 300) {
				selected.push_back(contours[i]);
				}
 		}
 		//cout << "Size of selected: " << selected.size() << endl;
 		/*for(unsigned int i = 0; i < selected.size(); i++){
 			Rect rec = boundingRect(selected[i]);
 			//cout << "looping on selected!" << endl;
 			rectangle(matResize, rec.br(), rec.tl(), BLACK);
 		}*/
 		//
 //			if there is only 1 target, then we have found the target we want
 		if(selected.size() == 1){
 			//cout << "selected is one!" << endl;
 			Rect rec = boundingRect(selected[0]);
 //				"fun" math brought to you by miss daisy (team 341)!
 			y = rec.br().y + rec.height / 2;
 			y= -((2 * (y / matResize.cols)) - 1);
 			//visionMutex.lock();
 			visionDistance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) /
 					tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * PI / 180);
 //				angle to target...would not rely on this
 			//targetX = rec.tl().x + rec.width / 2;
 			//targetX = (2 * (targetX / matResize.rows)) - 1;
 			//azimuth = normalize360(targetX*HORIZONTAL_FOV /2.0 + 0);
 //				drawing info on target
 			center =  Point(rec.br().x-rec.width/ 2 ,rec.br().y - rec.height / 2);
 			//Point centerw = Point(rec.br().x-rec.width / 2 - 15,rec.br().y - rec.height / 2 - 20);
 			//int distanceX = center.x;
 			//int distanceY = center.y;
 			//putText(matResize, to_string(distanceX), center, FONT_HERSHEY_PLAIN, 1, GREEN);
 			//putText(matResize, to_string(distanceY), centerw, FONT_HERSHEY_PLAIN, 1, GREEN);
 			cout << "Center: " << center << endl;
			int distanceCenter = getDistanceToCenter();
			int64_t t3 = getTickCount();
			double secs2 = (t3-t0) / getTickFrequency();
			cout << "Time to process image: " << secs2 << endl;
			visionTable->PutNumber("center", distanceCenter);
			visionTable->PutNumber("distance", visionDistance);
			visionTable->PutNumber("targetX", center.x);
 			//cout << "Distance: " << visionDistance << endl;
 			//visionMutex.unlock();
 		}
 		//imwrite("/home/lvuser/output.jpg", matResize);
 		//cvtColor(matResize, matGray, COLOR_BGR2GRAY, 0);
 		//imaqArrayToImage(myImaqImage, matGray.data, matGray.cols, matGray.rows);
 		//CameraServer::GetInstance()->SetImage(myImaqImage);
		int64_t t1 = getTickCount();
		double secs = (t1-t0)/getTickFrequency();
		cout << "Time to process image and send to network table: " << secs << endl;
 	}
}

void visionTest() {
	// TODO Auto-generated method stub
	matOriginal =  Mat();
	NetworkTable::SetIPAddress("10.20.53.2");
	NetworkTable::SetClientMode();
	NetworkTable::Initialize();
	visionTable = NetworkTable::GetTable("vision");
	matHSV =  Mat();
	matThresh =  Mat();
	clusters =  Mat();
	matHeirarchy =  Mat();
	matGray = Mat();
	matResize = Mat();
	matNEWRGB = Mat();
	testingMat = Mat();
	//myImaqImage = imaqCreateImage(ImageType::IMAQ_IMAGE_U8, 0);
    videoCapture =  VideoCapture();
	videoCapture.open(0);
//	main loop of the program
	while(!videoCapture.isOpened())
	{
		printf("videoCapture could not open!\n");
	}
	videoCapture.set(CV_CAP_PROP_EXPOSURE, -100);
	processImage();
	imwrite("/home/lvuser/output.jpg", matOriginal);
//	make sure the java process quits when the loop finishes
	videoCapture.release();
	exit(0);
}

int getTargetX() {
	return center.x;
}

int getTargetY() {
	return center.y;
}

float getDistanceY() {
	return visionDistance;
}

int getDistanceToCenter() {
	return centerOfCam.x - center.x;
}

int main( int argc, const char** argv )
{
    visionTest();
    return 0;
}
