/*
 * Estimates the depth of an RGB image using the neural network in MATLAB. Communication is done through the filesystem.
 * The resulting depth map is broadcasted using a ROS publisher to the other nodes
 */

 // ROS includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

image_transport::Publisher depth_pub;

std::string communication_folder = "/home/jay/data/testTMP";
std::string path_RGB_image;
std::string im_ready_path;
std::string depth_ready_path;
std::string depth_estim_path;

void publish_depth_estim(cv::Mat depth_estim) {
	ros::Time time = ros::Time::now();
  cv_bridge::CvImage cvi;
  cvi.header.stamp = time;
  cvi.encoding = sensor_msgs::image_encodings::BGR8;
  cvi.image = depth_estim;

  depth_pub.publish(cvi.toImageMsg());
}

void callbackWithoutCameraInfoWithDepth(const sensor_msgs::ImageConstPtr& original_image) {
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

	//C opy the image.data to imageBuf. 
	cv::Mat rgb_image = cv_ptr->image;
	// Write to file
	std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(100);
	cv::imwrite(path_RGB_image, rgb_image, compression_params);
	// Write a file that signifies the jpeg is ready
	FILE * fid = fopen(im_ready_path.c_str(), "w");
	fclose(fid);
	// Wait until ready file exists
	while (!boost::filesystem::exists(depth_ready_path)) {}
	boost::filesystem::remove(depth_ready_path);
	// Read file
	cv::Mat depth_estim = cv::imread(depth_estim_path,CV_LOAD_IMAGE_GRAYSCALE );

	publish_depth_estim(depth_estim);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "depth_estimator_node", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	image_transport::Subscriber sub_image = it.subscribe(
			"/ardrone/image_raw", 1, callbackWithoutCameraInfoWithDepth);

	depth_pub = it.advertise("/autopilot/depth_estim", 100);

	// Get location for the files
	nh.getParam("depth_estimation_path", communication_folder);

	path_RGB_image= communication_folder + "/image.jpg";
	im_ready_path= communication_folder + "/imageReady";
	depth_ready_path= communication_folder + "/depthReady";
	depth_estim_path= communication_folder + "/depth.jpg";

	ros::spin();

}
