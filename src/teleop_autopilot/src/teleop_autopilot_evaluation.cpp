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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <unistd.h>

#include <std_srvs/Empty.h>
#include <algorithm>


#include <fstream>      // std::ifstream
#include <sstream>     // std::cout
using namespace std;

string online_control="";
string supervised_control="";
string save_log_location="/home/jay/autopilot_ws/src/autopilot/log.txt";
float count_average=0.001;
float total_average=0;
float running_average=0;
float running_width=100;

float close_threshold=0.41;

bool takeoff=false;
bool shuttingdown=false;
int wait_first_images=10;//dont evaluate the first depth images as the kinnect is still starting up...


void shutdown(string msg){
  std::cout << "SHUTDOWN ################################################ :"<< msg << std::endl;
  shuttingdown=true;
  //write the message away to now which one is success and which one failures
  ofstream res_file(save_log_location.c_str(), std::ios_base::app);
  if(res_file){
    res_file << msg << "\n" ;
    res_file.close();
  }
  stringstream command;
  int pid;
  ifstream file("/home/jay/autopilot_ws/src/autopilot/.pid");
  if(file)
  {
    stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    buffer >> pid;
  }
  
  command << "gnome-terminal -x sh -c 'kill -9 "<< pid <<"'";
  string command_str = command.str();
  cout << command_str.c_str() << endl;
  system(command_str.c_str());
  usleep(5000);
}


/** Class to deal with which callback to call whether we have CameraInfo or not
 */
class Callbacks {
public://initialize fields of callbacks
  string path;
  string path_depth;
  Callbacks() : is_first_image_depth_(true), count_(0){}
  
  //listen to the online control
  void callbackOnlineControl(geometry_msgs::Twist msg)
  {
    geometry_msgs::Vector3 lin = msg.linear;
    geometry_msgs::Vector3 ang = msg.angular;
    //std::cout << "twist message: "<< msg << std::endl;
    //std::cout << "twist message ang z: "<< ang.z << std::endl;
    
    std::string cmd;
    //check stop signal if quadrotor has to hover
    if( !(lin.x || lin.y || lin.z || ang.z)){
      cmd = "100000000";
    }else{
      if(lin.y < 0){
	cmd = "000100000";
      }else if(lin.y > 0){
	cmd = "000010000";
      }else{
	if(lin.x < 0){
	  cmd = "010000000";
	}else if(lin.x > 0){
	  cmd = "001000000";
	}else{
	  if(lin.z < 0){
	    cmd =  "000001000";
	  }else if(lin.z > 0){
	    cmd =  "000000100";
	  }else{
	    if(ang.z < 0){
	      cmd =  "000000010";
	    }else if(ang.z > 0){
	      cmd =  "000000001";
	    }else{
	      cmd = "x";
	    }
	  }
	}
      }
    }
    online_control = cmd;
    std::cout << "online control: "<< online_control << std::endl;
  }

  //listen to the supervised control
  void callbackDaggerControl(geometry_msgs::Twist msg)
  {
    geometry_msgs::Vector3 lin = msg.linear;
    geometry_msgs::Vector3 ang = msg.angular;
    //std::cout << "twist message: "<< msg << std::endl;
    //std::cout << "twist message ang z: "<< ang.z << std::endl;
    
    std::string cmd;
    //check stop signal if quadrotor has to hover
    if( !(lin.x || lin.y || lin.z || ang.z)){
      cmd = "100000000";
    }else{
      if(lin.y < 0){
	cmd = "000100000";
      }else if(lin.y > 0){
	cmd = "000010000";
      }else{
	if(lin.x < 0){
	  cmd = "010000000";
	}else if(lin.x > 0){
	  cmd = "001000000";
	}else{
	  if(lin.z < 0){
	    cmd =  "000001000";
	  }else if(lin.z > 0){
	    cmd =  "000000100";
	  }else{
	    if(ang.z < 0){
	      cmd =  "000000010";
	    }else if(ang.z > 0){
	      cmd =  "000000001";
	    }else{
	      cmd = "x";
	    }
	  }
	}
      }
    }
    supervised_control = cmd;
    std::cout << "supervised control: "<< supervised_control << std::endl;
  }

  //general callback function for the depth map
  void callbackDepth(const sensor_msgs::ImageConstPtr& original_image)
  {
    if(shuttingdown || !takeoff) return;
    if (is_first_image_depth_) {
      is_first_image_depth_ = false;
      // Wait a tiny bit to see whether callbackWithCameraInfo is called
      ros::Duration(0.001).sleep();
    }
    if (count_ < wait_first_images ){
      count_ ++;
      return;
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

    //Copy the image.data to imageBuf. Depth image is uint16 with depths in mm.
    cv::Mat depth_float_img = cv_ptr->image;
    double min, max;
    cv::minMaxLoc(depth_float_img, &min, &max);
    // cout << "min: " << min <<", max: "<<max <<endl;
    if( min < close_threshold ){
      //BUMP!
      string msg="bump";
      shutdown(msg);
    }
  }
  //set takeoff true so the node can start working
  void callbackTakeoff(std_msgs::Empty msg)
  {
    if(!takeoff) takeoff=true;
  }
  //use groundtruth to know if the quad reached the end of the trajectory: 18
  void callbackGt(const nav_msgs::Odometry& msg)
  {
    if(shuttingdown) return;
    //std::cout << "y-direction: "<< msg.pose.pose.position.y << std::endl;
    //Check from groundtruth whether drone is in the end of the trajectory
    if (msg.pose.pose.position.y >= 16 && !shuttingdown){
      shutdown("success");
    }
  }

private: //private fields of callback
  bool is_first_image_depth_;
  bool has_camera_info_;
  size_t count_;
  string control;
};

//compare the supervised_control and the online_control and keep a running average
void evaluate(){
  if(!takeoff) return;
  //check if supervised control is equal to estimated control
  int result = 0;
  if(online_control.compare(supervised_control)==1)result = 1;
  //update total average
  total_average=total_average+(result-total_average)/count_average;
  //update running average
  running_average=running_average+(result-running_average)/running_width;
  
  cout << "total average: "<<total_average<<". running average "<<running_average<< " over last "<<running_width<<" frames."<<endl;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_autopilot", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  std::string topic_depth = "/ardrone/kinect/depth/image_raw";
  
  //Get save_log_location BUG still needs to be tested!
  //if(nh.resolveName("log").c_str() != "") save_log_location = nh.resolveName("log");
  /*if(nh.resolveName("log").c_str() != "log"){
    save_log_location = nh.resolveName("log");
    save_log_location = save_log_location +".txt";
  }*/
  Callbacks callbacks;
  
  // Useful when CameraInfo is not being published
  //depth
  image_transport::Subscriber sub_image_depth = it.subscribe(
      topic_depth, 1, boost::bind(&Callbacks::callbackDepth, &callbacks, _1));

  
  // Make subscriber to ground_truth in order to get the psotion.
  ros::Subscriber subGroundtruth = nh.subscribe("/ground_truth/state",1,&Callbacks::callbackGt, &callbacks);
  
  // Subscribe to the takeoff message in order to know when to start the evaluation
  ros::Subscriber subTakeoff = nh.subscribe("/ardrone/takeoff",1,&Callbacks::callbackTakeoff, &callbacks);
  
  // Make subscriber to online control estimate from the network
  //ros::Subscriber subOnlineControl = nh.subscribe("/cmd_vel",1,&Callbacks::callbackOnlineControl, &callbacks);
  
  // Make subscriber to supervised control
  //ros::Subscriber subDaggerControl = nh.subscribe("/dagger_vel",1,&Callbacks::callbackDaggerControl, &callbacks);
  
  ros::Rate loop_rate(20);

  while(ros::ok()){
    //evaluate();
    loop_rate.sleep();
    ros::spinOnce();
  }
} 
