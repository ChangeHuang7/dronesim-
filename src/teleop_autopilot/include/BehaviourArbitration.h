#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>

class BehaviourArbitration
{
public:
	BehaviourArbitration();
	~BehaviourArbitration();

	float avoidObstacleHorizontal(cv::Mat kinectImage, float currentBearing);
	float avoidObstacleVertical(cv::Mat kinectImage, float currentBearing);

	float followGoal(float goalAngle, float currentBearing);

	float sumBehavioursHorz(float angVelAvoidHorz, float angVelFollowGoal);
	float sumBehavioursVert(float angVelAvoidHorz, float angVelFollowGoal);

	/* data */
private:
	void displayObstacleArrayHorz(cv::Mat obstacleMap);
	void displayObstacleArrayVert(cv::Mat obstacleMap);
	/* data */

	/* Horizontal control constants */
	float lambdaGoalHorz, lambdaObstacleHorz;
	float weightGoalHorz, weightObstacleHorz;
	// c_obst in paper
	float obstacleDistanceGainHorz;
	// sigma_i in paper
	float angularRangeHorz;

	/* Vertical control constants */
	float lambdaObstacleVert, angularRangeVert, obstacleDistanceGainVert;

	// The random number generator
};