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

#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <iostream>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

// #define RECEIVE_DEPTH_MAP

image_transport::Publisher depth_pub;

std::string communication_folder = "/home/jay/data/depth_estimation";
// std::string communication_folder_emerald = "/home/jay/data/rgb_emerald";
std::string communication_folder_emerald = "/home/jay/Desktop/";
std::string path_RGB_image;
std::string im_ready_path;
std::string depth_ready_path;
std::string depth_estim_path;

int SOCKET;
int PORTNO;
int COUNT =0;
using namespace std;
// using namespace cv;

void publish_depth_estim(cv::Mat depth_estim) {
	ros::Time time = ros::Time::now();
  cv_bridge::CvImage cvi;
  cvi.header.stamp = time;
  cvi.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  cvi.image = depth_estim;
  std::cout << "Publishing image" << std::endl;
  depth_pub.publish(cvi.toImageMsg());
}

void startImageReceiver() {
	int sockfd;
	socklen_t clilen;
	struct sockaddr_in serv_addr, cli_addr;
	// Open socket: domain, type, protocol (when 0, OS chooses appropriate protocol)
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) 
		ROS_ERROR("ERROR opening socket");


	int enable = 1;
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
		ROS_ERROR("setsockopt(SO_REUSEADDR) failed");
	// Set buffer to zeros
	bzero((char *) &serv_addr, sizeof(serv_addr));
	// Change input argument to int

	// set fields to approptriate values
	serv_addr.sin_family = AF_INET;
	// Set server address to this machine
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	// Convert port number to network byte order (suing htons) and add to serv_addr
	serv_addr.sin_port = htons(PORTNO);

	// This binds the adress of the socket to the socket itself
	if (bind(sockfd, (struct sockaddr *) &serv_addr,
	      sizeof(serv_addr)) < 0) 
	      ROS_ERROR("ERROR on binding");
	// Needs to be called, otherwise the program can't accept any connections
	listen(sockfd,5);
	clilen = sizeof(cli_addr);
	SOCKET = accept(sockfd, 
	         (struct sockaddr *) &cli_addr, 
	         &clilen);
	if (SOCKET < 0) 
	  ROS_ERROR("ERROR on accept");
}

void sendImage(cv::Mat image) {
	// image = cv::imread("/home/jay/Desktop/depth.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	// cv::resize(image, image, cv::Size(640,360));
	cv::imshow("Sent image", image);
	// cv::waitKey(500);
	image = (image.reshape(0,1));
	int  imgSizeTransmit = image.total()*image.elemSize();
	cout << "Sending image " << COUNT << endl;
	cout << "Size: " << image.size() << "Type: " << image.type() <<  "Bytes " << imgSizeTransmit << endl;
	COUNT++;
	write(SOCKET, image.data, imgSizeTransmit);
  cout << "Image sent" << endl;

}

cv::Mat receiveImage() {

  cv::Mat img = cv::Mat::zeros( 360,640, CV_8UC1);

  int imgSize = img.total()*img.elemSize();
  uchar sockData[imgSize];
	//Receive data here
  int bytes = 0;
  for (int i = 0; i < imgSize; i += bytes) {
    if ((bytes = recv(SOCKET, sockData +i, imgSize  - i, 0)) == -1) {
      //std::cerr("recv failed", 1);
      }
   }

	// Assign pixel value to img

  int ptr=0;
  for (int i = 0;  i < img.rows; i++) {
      for (int j = 0; j < img.cols; j++) {                                     
          img.at<uchar>(i,j) = (sockData[ptr]);
          // img.at<cv::Vec3b>(i,j) = cv::Vec3b(sockData[ptr+ 0],sockData[ptr+1],sockData[ptr+2]);
          ptr=ptr+1;
      }
  }
  cv::imshow("Received", img);
  cv::waitKey(10);
  cout << "Depthmap received!" << endl;

  // write(SOCKET,"I got your message",18);

  return img;
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
	cout << "Size: " << rgb_image.size() << "Type: " << rgb_image.type() << endl;

	sendImage(rgb_image);

	#ifdef RECEIVE_DEPTH_MAP
		cv::Mat depth_estim = receiveImage();

		// cv::Mat depth_estim = cv::imread("/home/jay/Desktop/depth.jpg", CV_LOAD_IMAGE_GRAYSCALE);

		// Write to file
		// std::vector<int> compression_params;
	 //  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	 //  compression_params.push_back(100);
		// cv::imwrite(path_RGB_image, rgb_image, compression_params);
		// // Write a file that signifies the jpeg is ready
		// FILE * fid = fopen(im_ready_path.c_str(), "w");
		// fclose(fid);
		// chmod(im_ready_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

		// // Wait until ready file exists
		// while (!boost::filesystem::exists(depth_ready_path)) {
		// 	if(!boost::filesystem::exists(im_ready_path)) {
		// 		FILE * fid = fopen(im_ready_path.c_str(), "w");
		// 		fclose(fid);
		// 		chmod(im_ready_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
		// 	}
		// 	ros::Duration(0.001).sleep();

		// }
		// boost::filesystem::remove(depth_ready_path);
		// boost::filesystem::remove(im_ready_path);

		// // Read file
		// cv::Mat depth_estim = cv::imread(depth_estim_path,CV_LOAD_IMAGE_GRAYSCALE );

		
		cv::Mat_<float> depth_estim_float, depth_estim_float_resized;

		depth_estim.convertTo(depth_estim_float, CV_32F);
		cv::resize(depth_estim_float,depth_estim_float_resized,cv::Size(640,360));


		// Scale matrix to fit between 0 and 5
		publish_depth_estim(depth_estim_float_resized);

		// Wait for a while, so the MATLAB script can catch up
		ros::Duration(0.005).sleep();
	#endif
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

	nh.param("depth_TCP_port", PORTNO, 55555);
	cout << "Using port " << PORTNO << endl;

	path_RGB_image= communication_folder_emerald + "/image.jpg";
	im_ready_path= communication_folder_emerald + "/imageReady";
	depth_ready_path= communication_folder_emerald + "/depthReady";
	depth_estim_path= communication_folder_emerald + "/depth.jpg";

	startImageReceiver();

	// Make sure no files are currently in the folder
	// boost::filesystem::remove(path_RGB_image);
	// boost::filesystem::remove(im_ready_path);
	// boost::filesystem::remove(depth_ready_path);
	// boost::filesystem::remove(depth_estim_path);

	ros::spin();

}
