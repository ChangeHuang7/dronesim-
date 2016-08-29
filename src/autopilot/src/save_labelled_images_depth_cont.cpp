/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**
 * Connect the image saving node to cmd_vel control node so control is written away into name of the image file: [-x +x -y +y -z +z cw ccw stop].
    $ rosrun autopilot save_labelled_images generated_set=”mydata”
    Saves images in /home/dell/data/mydata while listening to control of teleop node:
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    
    Scaling depth image from range 0.4m till 3m into jpg of values between 0..255 (alpha = 80)
*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <geometry_msgs/Twist.h>
#include <stdexcept>
#include <std_srvs/Empty.h>
#include <algorithm>
#include <std_msgs/Empty.h>

#include <fstream>      // std::ifstream
#include <sstream>     // std::cout

#include <sys/stat.h>
using namespace std;
bool overwrite = true;//if a folder with the same name exists, overwrite this folder.
string save_log_location;
string control_output_filename="";

bool depth_estimation_flag;
bool takeoff=false;
bool shuttingdown=false;
int max_count=10000;
boost::format g_format;
bool save_all_image, save_image_service;
std::string encoding;
bool service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  save_image_service = true;
  return true;
}


/** Class to deal with which callback to call whether we have CameraInfo or not
 */
class Callbacks {
public://initialize fields of callbacks
  string path;
  //string path="/home/dell/remote_images";
  string path_depth;
  Callbacks() : is_first_image_(true), is_first_image_depth_(true), has_camera_info_(false), count_(0){}
  
  void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
    //Process images: 
    if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
      mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
    float alpha = 256/4.6;
    float beta = -2.7*256/4.6+128;
    cv::convertScaleAbs(float_img, mono8_img, alpha, beta);
  }
  
  void callbackWithoutCameraInfo(const sensor_msgs::ImageConstPtr& image_msg)
  {
    if (is_first_image_) {
      is_first_image_ = false;

      // Wait a tiny bit to see whether callbackWithCameraInfo is called
      ros::Duration(0.001).sleep();
    }
    
    if (!takeoff)//dont save any image if there has no control signal arrived just yet
      return;
    // save the image
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(image_msg, encoding)->image;
    } catch(cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to bgr8", image_msg->encoding.c_str());
      return;
    }
    std::string filename;
    if (!saveImage(image, filename))
	return;
    count_++;
    //if(static_cast<int>(count_) >= max_count) shutdown("stuck");
        
  }

  void callbackWithoutCameraInfoWithDepth(const sensor_msgs::ImageConstPtr& original_image)
  {
    if (is_first_image_) {
      is_first_image_depth_ = false;
      // Wait a tiny bit to see whether callbackWithCameraInfo is called
      ros::Duration(0.001).sleep();
    }
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
    if (!takeoff)//dont save any image if there has no control signal arrived just yet
      return;
    //Copy the image.data to imageBuf. Depth image is uint16 with depths in m.
    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat depth_mono8_img;
    depthToCV8UC1(depth_float_img, depth_mono8_img);
//     double min, max;
//     cv::minMaxLoc(depth_float_img, &min, &max);	
//     std::cout << "minmax float: " << min << "; " << max << "\n";
//     cv::minMaxLoc(depth_mono8_img, &min, &max);	
//     std::cout << "minmax mono: " << min << "; " << max << "\n";
//     
    // save the image
    std::string filename;
    if (!saveImage(depth_mono8_img, filename, true))
	return;
    count_++;
    //if(static_cast<int>(count_) >= max_count) shutdown("stuck");
  }
  
  void callbackTakeoff(std_msgs::Empty msg)
  {
    if(!takeoff) takeoff=true;
  }
    
  void callbackCmd(const geometry_msgs::Twist& msg)
  {
    latest_twist = msg; 
  }
  
private: //private methods of callback
  bool saveImage(cv::Mat image, std::string &filename, bool depth = false) {
    if (!image.empty()) {
      try {
        filename = (g_format).str();
      } catch (...) { g_format.clear(); }
      try {
        filename = (g_format % path).str();
      } catch (...) { g_format.clear(); }
      try {
        filename = (g_format % path % count_).str();
      } catch (...) { g_format.clear(); }
      try { 
        filename = (g_format % path % count_ % "jpg").str();
      } catch (...) { g_format.clear(); }
      try {
      	if(!depth){
      	  filename = (g_format % path % count_ % "jpg").str();
      	}else{
      	  filename = (g_format % path_depth % count_ % "jpg").str();
      	}
      } catch (...) { g_format.clear(); }
      
      if ( save_all_image || save_image_service ) {
        try{
          cout << "Filename: " << filename << endl;

          if (depth_estimation_flag && image.channels() == 1) {
            double min;
            double max;
            cv::minMaxIdx(image, &min, &max);
            cv::Mat adjMap;
            // expand your range to 0..255. Similar to histEq();
            image.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 

            cv::Mat dst;
            cv::applyColorMap(adjMap, dst, cv::COLORMAP_JET);
            cv::imshow("Depth estimation",image);
            cv::imwrite(filename, dst);
            cv::waitKey(1);
          }
          else {
      	    cv::imwrite(filename, image); 
          }
          writeVelInfo();
      	  ROS_INFO("Saved image %s", filename.c_str());

      	  save_image_service = false;
      	}catch(runtime_error& ex){
      	  fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
      	  return false;
      	}
      } else {
        return false;
      }
    } else {
      ROS_WARN("Couldn't save image, no data!");
      return false;
    }
    return true;
  }

  /* Writes velocity (control) info to file */
  void writeVelInfo() {
    ofstream control_output_file;
    control_output_file.open(control_output_filename.c_str(), ios::app);
    char countArray[11];
    sprintf(countArray, "%010d", (int) count_);
    control_output_file << countArray << " " << latest_twist.linear.x << " " << latest_twist.linear.y << " " << latest_twist.linear.z
                            << " " << latest_twist.angular.x << " " << latest_twist.angular.y << " " << latest_twist.angular.z << endl;
    control_output_file.close();

  }

private: //private fields of callback
  bool is_first_image_;
  bool is_first_image_depth_;
  bool has_camera_info_;
  size_t count_;//size_t
  // string control;
  geometry_msgs::Twist latest_twist;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  //std::string topic = nh.resolveName("image");
//  std::string topic = "/ardrone/image_raw";
  std::string topic;
  std::string topic_depth;
  depth_estimation_flag = false;
  nh.getParam("depth_estimation_running", depth_estimation_flag);
  if (depth_estimation_flag) {
    topic = "/ardrone/image_raw";
    topic_depth = "/autopilot/depth_estim";
  }
  else {  
    topic = "/ardrone/kinect/image_raw";
    topic_depth = "/ardrone/kinect/depth/image_raw";
  }

  Callbacks callbacks;
  
  //callbacks.control ="10000000";
  //callbacks.path = "/home/jay/data/";

  //obtain saving location
  std::string saving_location;
  if(!nh.getParam("saving_location", saving_location)) {
    // if getParam returns falls, parameter was not set, don't save images!
    ROS_ERROR("No saving directory given, not saving images!");
    // Shutdown this node
    ros::shutdown();
    // Stop running
    exit(0);
  } //nh.resolveName("generated_set");

  // // Get the filepath of the log
  // if(!nh.getParam("saving_location_log", save_log_location)) {
  //   ROS_ERROR("No saving directory given for the log!");
  //   // Shutdown this node
  //   ros::shutdown();
  //   // Stop running
  //   exit(0);
  // }
  // else {
  //   cout << "Log path: " << save_log_location << endl;
  // }
  
  //if(saving_location.compare("generated_set")) saving_location = "remote_images/set_online";
  control_output_filename = "/home/jay/data/"+saving_location+"/control_info.txt";
  callbacks.path = "/home/jay/data/"+saving_location;
  boost::filesystem::path dir(callbacks.path);
  boost::filesystem::file_status f = status(dir);
  if(boost::filesystem::is_directory(f)){//directory exists
    if(!overwrite){
      //check if the subfolders exists both for depth and RGB
      callbacks.path = "/home/jay/data/"+saving_location+"/RGB";
      boost::filesystem::path dir_sub(callbacks.path);
      boost::filesystem::file_status f_sub = status(dir_sub);
      if(! boost::filesystem::is_directory(f_sub)){
	if(boost::filesystem::create_directory(dir_sub)) {
	  std::cout << "Success in creating: "<<callbacks.path << "\n";
	}else{
	  std::cout <<"Failed to make saving direction "<<callbacks.path <<"\n";
	  throw std::invalid_argument( "[save_labelled_images_depth]Failed to make saving direction" );
	}
      }else{//file already exists but we are not allowed to overwrite.
	std::cout <<"[save_labelled_images_depth] Directory already exists and overwrite is false... "<<callbacks.path <<"\n";
	throw std::invalid_argument( "[save_labelled_images_depth] Directory already exists and overwrite is false..." );
      }
      callbacks.path_depth = "/home/jay/data/"+saving_location+"/depth";
      dir_sub = callbacks.path_depth;
      f_sub = status(dir_sub);
      if(! boost::filesystem::is_directory(f_sub)){
	if(boost::filesystem::create_directory(dir_sub)) {
	  std::cout << "Success in creating: "<<callbacks.path_depth << "\n";
	}else{
	  std::cout <<"Failed to make saving direction "<<callbacks.path_depth <<"\n";
	  throw std::invalid_argument( "[save_labelled_images_depth]Failed to make saving direction" );
	}
      }else{
	throw std::invalid_argument( "[save_labelled_images_depth]Directory already exists and overwrite is false..." );
      }
    }else{//if overwrite => remove the folders
      boost::filesystem::remove_all(dir);
    }
  }//Create new folders if they dont exist.
  f = status(dir);
  if(! boost::filesystem::is_directory(f)){
    if(boost::filesystem::create_directory(dir)) {
      //change 
      chmod(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
      callbacks.path = "/home/jay/data/"+saving_location+"/RGB";
      boost::filesystem::path dir_rgb(callbacks.path);
      if(boost::filesystem::create_directory(dir_rgb)) {
	std::cout << "Success in creating: "<<callbacks.path << "\n";
      }
      //change permissions
      chmod(callbacks.path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
      callbacks.path_depth = "/home/jay/data/"+saving_location+"/depth";
      boost::filesystem::path dir_depth(callbacks.path_depth);
      if(boost::filesystem::create_directory(dir_depth)) {
	std::cout << "Success in creating: "<<callbacks.path_depth << "\n";
      }
      //change permissions
      chmod(callbacks.path_depth.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    }else{
      std::cout <<"Failed to make saving direction "<<callbacks.path <<"\n";
      throw std::invalid_argument( "[save_labelled_images_depth]Failed to make saving direction" );
    }
  }
  // Useful when CameraInfo is not being published
  image_transport::Subscriber sub_image = it.subscribe(
      topic, 1, boost::bind(&Callbacks::callbackWithoutCameraInfo, &callbacks, _1));
  //depth
  image_transport::Subscriber sub_image_depth = it.subscribe(
      topic_depth, 1, boost::bind(&Callbacks::callbackWithoutCameraInfoWithDepth, &callbacks, _1));

  // Make subscriber to cmd_vel in order to set the name.
  ros::Subscriber subControl = nh.subscribe("/cmd_vel",1,&Callbacks::callbackCmd, &callbacks);
  // [hover, back, forward, turn right, turn left, down, up, clockwise, ccw]
  // Adapt name instead of left0000.jpg it should be 00000-gt1.jpg when receiving control 1 ~ straight
  ros::NodeHandle local_nh("~");
  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("%s/%010i.%s"));
  local_nh.param("encoding", encoding, std::string("bgr8"));
  local_nh.param("save_all_image", save_all_image, true);
  g_format.parse(format_string);
  ros::ServiceServer save = local_nh.advertiseService ("save", service);
  
  // Subscribe to the takeoff message in order to know when to start the saving procedure
  ros::Subscriber subTakeoff = nh.subscribe("/ardrone/takeoff",1,&Callbacks::callbackTakeoff, &callbacks);
  
  ros::spin();
} 
