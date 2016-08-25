
#include "BehaviourArbitration.h"

// ROS includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>

#include <std_msgs/Empty.h>
using namespace std;

bool takeoff = false;
int FSM_COUNTER_THRESH=20;//wait for some time before taking off
int counter = 0;


float ADJUST_HEIGHT_MAX = 3.5; float ADJUST_HEIGHT_MIN = 0.5;
float adjust_height = 0;
float CURRENT_YAW = 0;
double GOAL_ANGLE = 3*CV_PI/2;
float DYAW = 0, DPITCH = 0;

BehaviourArbitration BAController;
ros::Publisher debugPub;

void updateController(cv::Mat depth_float_img) {
	float dYawObstacle = BAController.avoidObstacleHorizontal(depth_float_img, CURRENT_YAW);
	float dYawGoal 	   = BAController.followGoal(GOAL_ANGLE, CURRENT_YAW);
	DYAW               = BAController.sumBehavioursHorz(dYawObstacle, dYawGoal);
	DPITCH             = BAController.avoidObstacleVertical(depth_float_img, CURRENT_YAW);

	// DYAW = 0.5;
	std_msgs::Float32 msg;
	msg.data = CURRENT_YAW;
	debugPub.publish((msg));
	cout << "Angular velocity : " << DYAW << endl;
	cout << "DPitch: " << DPITCH << endl;
}

//Callback function for the estimated depth
void callbackDepthEstim(const sensor_msgs::ImageConstPtr& original_image) {
	cv_bridge::CvImagePtr cv_ptr;
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image);
	}
	catch (cv_bridge::Exception& e)
	{
  		//if there is an error during conversion, display it
		ROS_ERROR("save_labelled_images_depth::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}

	//Copy the image.data to imageBuf. Depth image is uint16 with depths in mm.
	cv::Mat depth_float_img = cv_ptr->image;

	updateController(depth_float_img);
}

//general callback function for the depth map
void callbackWithoutCameraInfoWithDepth(const sensor_msgs::ImageConstPtr& original_image)
{
	// if (is_first_image_) {
	// 	is_first_image_depth_ = false;
	// 	// Wait a tiny bit to see whether callbackWithCameraInfo is called
	// 	ros::Duration(0.001).sleep();
	// }

	// if (has_camera_info_)
	// 	return;

	cv_bridge::CvImagePtr cv_ptr;
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image);
	}
	catch (cv_bridge::Exception& e)
	{
  		//if there is an error during conversion, display it
		ROS_ERROR("save_labelled_images_depth::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}

	//Copy the image.data to imageBuf. Depth image is uint16 with depths in mm.
	cv::Mat depth_float_img = cv_ptr->image;

	for(int row = 0; row < depth_float_img.rows; row++) {
    	for(int col = 0; col < depth_float_img.cols; col++) {
      		if (isnan(depth_float_img.at<float>(row,col))) {
		        // Set to big enough number
		        // cout << "not a number" << endl;
		        depth_float_img.at<float>(row,col) = 5;
      		}
    	}
  	}
	updateController(depth_float_img);
}

double getYaw(geometry_msgs::Quaternion orientation) {
	double yaw = tf::getYaw(orientation);
	return yaw;
}

void callbackGt(const nav_msgs::Odometry& msg)
{
	if (msg.pose.pose.position.z > ADJUST_HEIGHT_MAX){
		adjust_height=-1;
	}else if (msg.pose.pose.position.z < ADJUST_HEIGHT_MIN){ // Was 0.5
		adjust_height=1;
	}else{
		adjust_height=0;
	}

	CURRENT_YAW = getYaw(msg.pose.pose.orientation) + CV_PI;
	// cout << "Yaw" << CURRENT_YAW << endl;
}

geometry_msgs::Twist get_twist() {
	//take off after FSM counter greater than the threshold
	if(counter > FSM_COUNTER_THRESH) takeoff=true;
  
	geometry_msgs::Twist twist;

	twist.linear.x = 0.8;
	twist.linear.y = 0.0;
	twist.linear.z = DPITCH + adjust_height;

	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = DYAW;
	
	
	counter=counter+1;
	return twist;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_autopilot", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	//std::string topic = nh.resolveName("image");
	std::string topic_depth = "/ardrone/kinect/depth/image_raw";
	std::string topic_estim_depth = "/autopilot/depth_estim";


	image_transport::Subscriber sub_image_depth;
	// Use kinect by default
	bool use_depth_estim = false;
	nh.getParam("use_depth_estim", use_depth_estim);
	if (use_depth_estim) {
		cout << "Using depth estimation" << endl;
		sub_image_depth = it.subscribe(topic_estim_depth, 1, callbackDepthEstim);
	}
	else {
		cout << "Using kinect data" << endl;
		sub_image_depth = it.subscribe(
			topic_depth, 1, callbackWithoutCameraInfoWithDepth);
	}


	// Make subscriber to ground_truth in order to get the psotion.
	//ros::Subscriber subControl = nh.subscribe("/ground_truth/state/pose/pose/position",1,&Callbacks::callbackGt, &callbacks);
	ros::Subscriber subControl = nh.subscribe("/ground_truth/state",1,callbackGt);

	// Make subscriber to cmd_vel in order to set the name.
	ros::Publisher pubControl = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	
	// Make Publisher to cmd_vel in order to set the velocity.
	ros::Publisher pubTakeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  
	ros::Rate loop_rate(10);

	debugPub = nh.advertise<std_msgs::Float32>("debug_autopilot", 1000);

	geometry_msgs::Twist twist;

	// Get the goal angle from the launch file
	if(!nh.getParam("goal_angle", GOAL_ANGLE)) {
		cout << "Using default angle, ";
	}
	cout << "Goal angle: " << GOAL_ANGLE << endl;
	// BAController = new BAController();

	// twist.linear.x = 0.5;//straight
	// twist.linear.y = 0.0;//left
	// twist.linear.z = adjust_height;//up

	// twist.angular.x = 0.05;
	// twist.angular.y = 0.0;
	// twist.angular.z = 0.0;
	//while(nh.ok()){
	while(ros::ok()){
		twist = get_twist();

		pubControl.publish(twist);
		if(takeoff){
		      std_msgs::Empty msg;
		      pubTakeoff.publish(msg);
		    }
		loop_rate.sleep();
		ros::spinOnce();
	}
} 
