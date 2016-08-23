
#include "BehaviourArbitration.h"

//The old Kinect has a depth image resolution of 320 x 240 pixels with a fov of 58.5 x 46.6
// degrees resulting in an average of about 5 x 5 pixels per degree. (see source 1)
//source: http://smeenk.com/kinect-field-of-view-comparison/

using namespace cv;
using namespace std;

cv::RNG rng( 0xFFFFFFFF );

template <typename T>
std::string to_string(T value)
{
  std::ostringstream os ;
  os << value ;
  return os.str() ;
}

BehaviourArbitration::BehaviourArbitration() {
	lambdaGoalHorz = 0.5;
	lambdaObstacleHorz = 5;
	weightGoalHorz = 0.3;
	weightObstacleHorz = 0.7;
	obstacleDistanceGainHorz = 0.2;
	angularRangeHorz = 29*CV_PI/180; // Range is +-29° 

	lambdaObstacleVert = 5;
	angularRangeVert = 27.5*CV_PI/180;
	obstacleDistanceGainVert = 0.2;

}


void BehaviourArbitration::displayObstacleArrayHorz(cv::Mat obstacleMap) {
	cv::Mat displayImage = Mat::zeros(200, obstacleMap.cols*6, CV_8UC3);
	int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  	double fontScale = 0.5;
	int thickness = 1;

	Point testPos(10, 70);
	for (int i = 0; i< obstacleMap.cols; i += 10) {
		string depthString = to_string(floor(obstacleMap.at<float>(0,i)));
		// cout <<"depth"<< depthString << endl;
		putText(displayImage, depthString, testPos, fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
		testPos += Point(50,0);
	}
	imshow("Depth info horizontal", displayImage);
	waitKey(1);

}

void BehaviourArbitration::displayObstacleArrayVert(cv::Mat obstacleMap) {
	cv::Mat displayImage = Mat::zeros(200, obstacleMap.cols*4, CV_8UC3);
	int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  	double fontScale = 0.5;
	int thickness = 1;

	Point testPos(obstacleMap.cols*2, 30);
	for (int i = 0; i< obstacleMap.cols; i += 8) {
		string depthString = to_string(floor(obstacleMap.at<float>(0,i)));
		// cout <<"depth"<< depthString << endl;
		putText(displayImage, depthString, testPos, fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
		testPos += Point(0,30);
	}
	imshow("Depth info vertical", displayImage);
	waitKey(1);

}

/*
 * Returns the angular velocity outputted by the behaviour arbitration scheme
 * This is the obstacle avoid behaviour, should be summed with heading goal
 */
float BehaviourArbitration::avoidObstacleHorizontal(cv::Mat kinectImage, float currentBearing) {
	int image_height = kinectImage.rows;
	int image_width  = kinectImage.cols;
	int centre_row = floor(image_height/2);
	int obstacle_bins = 64;

	// Create the obstacle array
	cv::Mat obstacleMap = Mat::zeros(1, obstacle_bins, CV_32F);
	// 64 bins ~~> sum of every 5 depth bins
	// 1 bin = 0.90625°
	// cout << "Width: " << image_width << endl;
	// imshow("Depth map", kinectImage);
	// waitKey(1);
	for (int i = 0; i < image_width; i++) {
		// Sum 5 depth pixels for each obstacle bin
		// cout << "i: " << floor(i/5) << endl;
		// cout << "Depth: " << kinectImage.at<float>(centre_row, i) << endl;
		obstacleMap.at<float>(0, floor(i/10)) += kinectImage.at<float>(centre_row, i);
	}
	displayObstacleArrayHorz(obstacleMap);
	float fObstacleTotal = 0;

	for (int i = 0; i < obstacle_bins; i++) {
		// obstacleBearing between -29° and 29°
		float obstacleBearing = i*58/64-29;
		// Convert to radians
		obstacleBearing = obstacleBearing * CV_PI / 180;
		// float bearingDifference = currentBearing - obstacleBearing;
		float bearingDifference = obstacleBearing;
		// cout << "bearingDifference: " << bearingDifference << endl;
		// cout << "Obstacle distance: " << obstacleMap.at<float>(0,i) << endl;

		// Calculate the exponents
		double bearingExponent = exp(-pow(bearingDifference,2)/(2*pow(angularRangeHorz,2)));
		double distanceExponent = exp(-obstacleDistanceGainHorz * obstacleMap.at<float>(0,i));
		// cout << "Bearing exponent " << bearingExponent << endl;
		// Add product to the total sum
		fObstacleTotal += bearingDifference * bearingExponent * distanceExponent;
	}
	// Multiply the total sum by the appropriate lambda
	// cout << "fObstacleTotal: " << fObstacleTotal << endl;
	fObstacleTotal *= lambdaObstacleHorz;
	// return 0;
	return fObstacleTotal;

}
float BehaviourArbitration::avoidObstacleVertical(cv::Mat kinectImage, float currentBearing) {
	int image_height = kinectImage.rows;
	int image_width  = kinectImage.cols;
	int centre_col = floor(image_width/2);
	int obstacle_bins = 36;

	cout << "Image height: " << image_height << endl;

	cv::Mat obstacleMap = Mat::zeros(1, obstacle_bins, CV_32F);

	for (int i = 0; i < image_height; i++) {
		obstacleMap.at<float>(0, floor(i/10)) += kinectImage.at<float>(i,centre_col);
	}
	displayObstacleArrayVert(obstacleMap);
	float fObstacleTotal = 0;

	for (int i = 0; i < obstacle_bins; i++) {
		// obstacleBearing between -23.5° and 23.5° [Vertical FOV]
		float obstacleBearing = i*47/36-23.5;
		// Convert to radians
		obstacleBearing = obstacleBearing * CV_PI / 180;
		// float bearingDifference = currentBearing - obstacleBearing;
		float bearingDifference = obstacleBearing;
		// cout << "Obstacle distance: " << obstacleMap.at<float>(0,i) << endl;

		// Calculate the exponents
		double bearingExponent = exp(-pow(bearingDifference,2)/(2*pow(angularRangeVert,2)));
		double distanceExponent = exp(-obstacleDistanceGainVert * obstacleMap.at<float>(0,i));
		// Add product to the total sum
		fObstacleTotal += bearingDifference * bearingExponent * distanceExponent;
	}

	fObstacleTotal *= lambdaObstacleVert;
	return fObstacleTotal;
}

float BehaviourArbitration::followGoal(float goalAngle, float currentBearing) {
	// return 0;
	return -lambdaGoalHorz * sin(currentBearing - goalAngle);
}

float BehaviourArbitration::sumBehavioursHorz(float angVelAvoidHorz, float angVelFollowGoal) {
	// return 0.5;
	float behaviourSum = weightObstacleHorz * angVelAvoidHorz + weightGoalHorz*angVelFollowGoal;
	behaviourSum += rng.gaussian(0.5);
	return behaviourSum;
}

float sumBehavioursVert(float angVelAvoidVert, float angVelFollowGoal) {
	return 0;
}


BehaviourArbitration::~BehaviourArbitration() {

}