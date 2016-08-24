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
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <unistd.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

#include <std_srvs/Empty.h>
#include <algorithm>

using namespace std;

enum STATES {HOVER = 0, OBSTACLE_AVOID = 1, OBSTACLE_AVOID_CORNER = 2, TURN_GOAL = 3, OBSTACLE_AVOID_VERT,
              HEIGHT_ADJUST_UP_DOWN, HEIGHT_ADJUST_DOWN_UP};

float VERY_CLOSE_OBSTACLE_DEPTH = 0.5, CLOSE_OBSTACLE_DEPTH = 1.5;
float TURN_GOAL_OBSTACLE_DEPTH = 2;
float VERTICAL_GRADIENT_THRESHOLD = 1.5, VERTICAL_MOVEMENT_DISTANCE = 3;

int FSM_COUNTER_HEIGHT_ADJUST = 0;
int FSM_COUNTER_THRESH_HEIGHT_ADJUST1 = 35;
int FSM_COUNTER_THRESH_HEIGHT_ADJUST2 = 110;//edited klaas 
int FSM_COUNTER_THRESH_HEIGHT_ADJUST3 = 145;


float ADJUST_HEIGHT_MAX = 1.5; float ADJUST_HEIGHT_MIN = 0.75;
//float ADJUST_HEIGHT_MAX = 4; float ADJUST_HEIGHT_MIN = 2.5;


bool takeoff=false;

double CURRENT_YAW = 0;
double YAW_MARGIN = 0.1;
float VERTICLE_DIR = 0;
int ADJUST_DIR = 0;
int state = 0; //define some states that override the general obstacle avoidance behavior: 0 ~ wait, 1 ~ do obstacle avoidance, 2 ~ turn for a random amount of time in 1 direction
int FSM_COUNTER_THRESH=100;//count by which the state goes to the next state in the FSM (0->1->2->1->...)
float turn=1;//turn in random state: should be 
int oa_dur = 100;//time it is in oa state
int rnd_dur = 10;//time it is in random state

int FSM_COUNTER=0;
int STEER_DIR = 0;//0 means go left, 1 means go straight, 2 means turn right
float STEER_SPEED = 1;//0 means no turning, 1 means turn left, -1 means turn right
int adjust_height = 0; //0 means no adjust, 1 means go up, -1 means go down
double INITIAL_DIR = +1.5 + CV_PI; // yaw angle 0 -> 2*pi

const int DEPTH_SCALE_FACTOR = 80;//40;
const float STEER_SPEED_SCALE=1;//0.005;

ros::Publisher debugPub;

cv::RNG rng( 0xFFFFFFFF );

boost::format g_format;
bool save_all_image, save_image_service;
std::string encoding;
bool service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  save_image_service = true;
  return true;
}

template <typename T>
std::string to_string(T value)
{
  std::ostringstream os ;
  os << value ;
  return os.str() ;
}

/** Class to deal with which callback to call whether we have CameraInfo or not
 */
class Callbacks {
public://initialize fields of callbacks
string path;
  //string path="/home/dell/remote_images";
string path_depth;
Callbacks() : is_first_image_(true), is_first_image_depth_(true), has_camera_info_(false), count_(0){}

void depthToCV8UC1(cv::Mat& float_img, cv::Mat& mono8_img){
    //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);
  }
    //cv::convertScaleAbs(float_img, mono8_img, 80, 0.0);
  //Initialize m
  double minVal; 
  double maxVal; 
  cv::Point minLoc; 
  cv::Point maxLoc;

  cv::minMaxLoc( float_img, &minVal, &maxVal, &minLoc, &maxLoc );  // Iterate through all elements, to find huge negative values
  cout << "Min value: " << minVal << endl;
  cout << "Max value: " << maxVal << endl;
  // Which indicates out of range for kinect
  for(int row = 0; row < float_img.rows; row++) {
    for(int col = 0; col < float_img.cols; col++) {
      if (isnan(float_img.at<float>(row,col))) {
        // Set to big enough number
        // cout << "not a number" << endl;
        float_img.at<float>(row,col) = 5;
      }
    }
  }

  cv::convertScaleAbs(float_img, mono8_img, DEPTH_SCALE_FACTOR, 0.0);
  //The following doesn't work due to NaNs
  /*double min, max;
  cv::minMaxLoc(your_mat, &min, &max);	
  std::cout << min << "; " << max << */
  //double minVal, maxVal; 
  //minMaxLoc(float_img, &minVal, &maxVal);
  //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  //mono8_img = cv::Scalar(0);
  //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
  }
  
  //return the minimum out of 3 variables
  //used by getDirectionSteer
  int min_3_nums(int a, int b, int c)
  {
    if( a < b && a < c)
      return 0;
    else if( b < a && b < c)
      return 1;
    else if(c < a && c < b)
      return 2;
    /*
    if (a==b)
      return 0;
    if (b==c)
      return 1;*/
  }

  //calculate a proper steering direction and save this in the global variable steer_dir and steer_speed
  //the direction is picked from go straight/turn left/turn right depending on the depth image
void getDirectionSteer(cv::Mat depth_mono8_img, cv::Mat depth_float_img)
{
  int DEPTH_MAX = 255;

  int image_height = depth_mono8_img.rows;
  int image_width  = depth_mono8_img.cols;
  int num_depth_bins = image_width;//100;
  int stride_length    = floor(image_width/num_depth_bins);
  cv::Mat depth_vals(num_depth_bins,1,CV_32F);
  cv::Mat vector_field_hist = cv::Mat::zeros(DEPTH_MAX,num_depth_bins,CV_8UC3);
  
  int image_height_depth_val = floor(image_height/2);
  int image_col_val = floor(stride_length/2);

  
  //Build VFH
  for(int i = 0; i < num_depth_bins; ++i)
  {
    int depth_this = 0;
    int depth_scaled = 0;
    if( image_col_val <= image_width)
    {
    	depth_this = (int)(depth_mono8_img.at<unsigned char>(image_height_depth_val, image_col_val));  //(image_height_depth_val, image_col_val);
    }
    
    if (depth_this <=-1) // EXCEEDS MAX RANGE
    {
      depth_scaled = DEPTH_MAX;
      //cout << "Exceeds max " << depth_this << endl;
    }
    else if  (depth_this == 0) // LESS THAN MIN RANGE
    {
      depth_scaled = 0;
      //cout << "Less than min range " << depth_this << endl;
    }
    else {
      depth_scaled = depth_this;
      //cout << depth_scaled << endl;
    }
    
    depth_vals.at<float>(i,0) = (float)(depth_scaled);
    //cout << depth_this << " " << depth_scaled << endl;
    // cv::Point pt1, pt2;
    // pt1.x = i;
    // pt2.x = i;
    
    // pt1.y = DEPTH_MAX;
    // pt2.y = DEPTH_MAX-depth_scaled;
    
    // cv::line(vector_field_hist, pt1, pt2, cv::Scalar(0,0,255), 2, 8);
    
    image_col_val += stride_length;
  }
 //    // Find best steer direction
 //    int min_freespace_width = 30; //let this be an even number
 //    int freespace_min_depth = 255;
    
 //    cv::Mat free_space_widths = cv::Mat::zeros(num_depth_bins,1,CV_32F);
 //    //std::vector<int> free_space_directions;
 //    //std::vector<int> free_space_widths;
    
 //    for(int i = 0; i < num_depth_bins; ++i)//num_depth_bins-50
 //    {
 //      //cout << "i: " << i << endl;
 //      int depth_this = DEPTH_MAX;
 //      int left_offset = 0;
 //      while (depth_this >= freespace_min_depth && i-left_offset >= 0)
 //      {
	// //cout << "depth_this: " << depth_this << endl;
	// //cout << "left_offset: " << left_offset << endl;
 //       depth_this = (int)depth_vals.at<float>(i-left_offset,0);
 //       left_offset += 1;
 //     }
 //     depth_this = DEPTH_MAX;
 //     int right_offset = 0;

 //     while (depth_this >= freespace_min_depth && i+right_offset < num_depth_bins)
 //     {
	// //cout << "depth_this: " << depth_this << endl;
	// //cout << "right_offset: " << right_offset << endl;
 //       depth_this = (int)depth_vals.at<float>(i+right_offset,0);
 //       right_offset += 1;
 //     }
 //     float width = (float) (left_offset + right_offset);
 //     free_space_widths.at<float>(i,0) = left_offset + right_offset; 

 //     cv::circle(vector_field_hist, cv::Point(i,DEPTH_MAX-10), 3, cv::Scalar(0,0,(int)width), -1);

 //   }

 //   double min, max;
 //   cv::Point min_loc, max_loc;
 //   cv::minMaxLoc(free_space_widths, &min, &max, &min_loc, &max_loc);
 //   int best_free_space_direction = max_loc.y;
 //   int best_free_space_width     = (int)max;



    //cout << "Direction: " << best_free_space_direction << " Width: " << best_free_space_width << endl;

    //cv::imshow("vfh", vector_field_hist);
  float depths_left = 0, depths_right = 0, depths_centre = 0;

  int num_cols_third = (int)(num_depth_bins/3);
  for(int i = 0; i < num_cols_third; ++i)
  {
    // depths_left += depth_vals.at<float>(i,0);
    depths_left += depth_float_img.at<float>(image_height_depth_val, i);

  }
  depths_left /= num_cols_third;

  for(int i = num_cols_third; i < 2*num_cols_third; ++i)
  {
    // depths_centre += depth_vals.at<float>(i,0);
    depths_centre += depth_float_img.at<float>(image_height_depth_val, i);

  }
  depths_centre /= num_cols_third;

  for(int i = 2*num_cols_third; i < 3*num_cols_third; ++i)
  {
    // depths_right += depth_vals.at<float>(i,0);
    depths_right += depth_float_img.at<float>(image_height_depth_val, i);

  }
  depths_right /= num_cols_third;

    // Calculate depths high and low, along a line in the centre
  float depths_high = 0, depths_low = 0;
  int centre_col = floor(image_width/2);
  for (int i = 0; i < floor(image_height/2); i++) {
    depths_high += depth_float_img.at<float>(i, centre_col);
  }
  // Average value
  depths_high /= floor(image_height/2);

  for (int i = floor(image_height/2); i < image_height; i++) {
    depths_low += depth_float_img.at<float>(i,centre_col);
  }
  // Average value
  depths_low /= floor(image_height/2);

  /** DEBUG SECTION: can be safely commented out
      To comment this section, remove --> **/
  double minVal; 
  double maxVal; 
  cv::Point minLoc; 
  cv::Point maxLoc;
  // cout << depth_float_img << endl;


  cv::minMaxLoc( depth_float_img, &minVal, &maxVal, &minLoc, &maxLoc );  // Iterate through all elements, to find huge negative values
  // cout << "Min value: " << minVal << endl;
  // cout << "Max value: " << maxVal << endl;

  float origLeft = (depth_float_img.at<float>(image_height_depth_val, 10));
  float origCenter = (depth_float_img.at<float>(image_height_depth_val, (int)image_width/2));
  float origRight = (depth_float_img.at<float>(image_height_depth_val, image_width - 10));

  cv::rectangle(depth_mono8_img, cv::Point(0, image_height/2-100), cv::Point(image_width, image_height/2+2),
    cv::Scalar(0,0,0), -1);

  std::ostringstream stmLeft ;
  std::ostringstream stmCenter ;
  std::ostringstream stmRight ;
  stmLeft << depths_left ;

  string stextLeft = stmLeft.str();
  stmCenter << depths_centre ;

  string stextCenter = stmCenter.str();
  stmRight << depths_right ;
  string stextRight = stmRight.str();
  int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  double fontScale = 0.5;
  int thickness = 1;  
  cv::Point textLeft(10, 130);
  cv::Point textCenter(image_width/2-20, 130);
  cv::Point textRight(image_width-100, 130);
  cv::putText(depth_mono8_img, stextLeft, textLeft, fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
  cv::putText(depth_mono8_img, stextCenter, textCenter, fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
  cv::putText(depth_mono8_img, stextRight, textRight, fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
  cv::putText(depth_mono8_img, to_string(depths_low), textCenter + cv::Point(0,40), fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
  cv::putText(depth_mono8_img, to_string(depths_high), textCenter - cv::Point(0,20), fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);

  // cout << "left: " << depths_left << endl;
  // cout << "centre: " << depths_centre << endl;
  // cout << "right: " << depths_right << endl;

  std::ostringstream stmLeftOrig ;
  std::ostringstream stmCenterOrig ;
  std::ostringstream stmRightOrig ;
  stmLeftOrig << origLeft;
  stmCenterOrig << origCenter;
  stmRightOrig << origRight;

  cv::putText(depth_mono8_img, stmLeftOrig.str(), textLeft+cv::Point(0,20), fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
  cv::putText(depth_mono8_img, stmCenterOrig.str(), textCenter+cv::Point(0,20), fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
  cv::putText(depth_mono8_img, stmRightOrig.str(), textRight+cv::Point(0,20), fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);




  // cout << depth_vals << endl;
  cv::imshow("Mono depth map", depth_mono8_img);
  cv::waitKey(1);

  /** END DEBUG SECTION **/

  // double minVal; 
  // double maxVal; 
  // cv::Point minLoc; 
  // cv::Point maxLoc;
  // cout << depth_float_img << endl;


  cv::minMaxLoc( depth_float_img.row(image_height_depth_val), &minVal, &maxVal, &minLoc, &maxLoc );  // Iterate through all elements, to find huge negative values
  cout << "Max val in row: " << minVal << endl;

  // depths values: min: ~0.49, max: ~5
  // If you see object in the distance, with a gradient, go up/down
  float vert_gradient = depths_high - depths_low;
  std_msgs::Float32 msg;
  msg.data = vert_gradient;
  debugPub.publish((msg));
  cout << "Gradient: " << vert_gradient << endl;
  if (abs(vert_gradient) > VERTICAL_GRADIENT_THRESHOLD && depths_centre <= VERTICAL_MOVEMENT_DISTANCE) {
    state = OBSTACLE_AVOID_VERT;
    if (vert_gradient >= 0) {
      cout << "GO UP" << endl;
      state = HEIGHT_ADJUST_UP_DOWN;
    }
    else {
      cout << "GO DOWN" << endl;
      state = HEIGHT_ADJUST_DOWN_UP;
    }
  }
  // Obstacle very close
  else if ((depths_left <= VERY_CLOSE_OBSTACLE_DEPTH
    && depths_right <= VERY_CLOSE_OBSTACLE_DEPTH) 
    || depths_centre <= CLOSE_OBSTACLE_DEPTH)
  {
    cout << "GO LEFT " << endl;
    STEER_DIR = 0; //GO LEFT
  
    // cout << "GO STRAIGHT " << endl;
    //   STEER_DIR = 1; //GO STRAIGHT


    state = OBSTACLE_AVOID_CORNER;
    FSM_COUNTER = 0;
    // float rand_num = rng.uniform(0.f,1.f);
    // randomness for extricating from corners
    //cout << "rand num: " << rand_num << endl;
    /*if (rand_num >= 0.5)
    {
      STEER_SPEED = 1;
    }
    else
    {
      STEER_SPEED = -1;
    }*/
  
  }
  // Enough room to head to goal
  else if (minVal >= TURN_GOAL_OBSTACLE_DEPTH) {
    // Turn to goal if the yaw error is too large, else keep going forward
    if (abs(CURRENT_YAW - INITIAL_DIR) >  YAW_MARGIN) {
      cout << "TURN TO GOAL" << endl;
      state = TURN_GOAL;
    }
    else {
      cout << "GO STRAIGHT " << endl;
      STEER_DIR = 1; //GO STRAIGHT
      STEER_SPEED = 0;
      state = OBSTACLE_AVOID;
    }
  }
  // Enough room to move forward
  else if (depths_left >= CLOSE_OBSTACLE_DEPTH && depths_right >= CLOSE_OBSTACLE_DEPTH)
  {
    cout << "GO STRAIGHT " << endl;
    STEER_DIR = 1; //GO STRAIGHT
    STEER_SPEED = 0;
    state = OBSTACLE_AVOID;
  }
  // Move away from obstacle to the right
  else if (depths_left > depths_right)
  {
    cout << "GO LEFT " << endl;
    STEER_DIR = 0; //TURN LEFT
    STEER_SPEED = 1;
    state = OBSTACLE_AVOID;
     //float rand_num = rng.uniform(0.f,1.f);
    //cout << "rand num: " << rand_num << endl;
    // randomness for extricating from corners
    /*if (rand_num >= 0.5)
    {
      STEER_SPEED = 1;
    }
    else
    {
      STEER_SPEED = 0.5;
    }
  /*
      if (depths_left == 0)
      {
  STEER_DIR = 2; //TURN RIGHT
  STEER_SPEED = 1;
      }
      else
      {
  STEER_DIR = 0; //TURN LEFT
  STEER_SPEED = 1;
      }*/

  }
  // Move away from obstacle to the left
  else if (depths_right >= depths_left)
  {
    cout << "GO RIGHT " << endl;
    STEER_DIR = 2; //GO RIGHT
    STEER_SPEED = 1;
    state = OBSTACLE_AVOID;
  }


    
}
//general callback function for the depth map
void callbackWithoutCameraInfoWithDepth(const sensor_msgs::ImageConstPtr& original_image)
{
  if (state == OBSTACLE_AVOID_CORNER || state ==  HOVER || state == HEIGHT_ADJUST_UP_DOWN ||
    state == HEIGHT_ADJUST_DOWN_UP) return;
  if (is_first_image_) {
    is_first_image_depth_ = false;
    // Wait a tiny bit to see whether callbackWithCameraInfo is called
    ros::Duration(0.001).sleep();
  }

  if (has_camera_info_)
    return;

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
  cv::Mat depth_mono8_img;
  cv::Mat depth_mono8_img_unscaled;
  depthToCV8UC1(depth_float_img, depth_mono8_img);
  //cout << "Here" << endl;
  // cv::imshow("test_window", depth_mono8_img);
  // cv::waitKey(25);
  
  getDirectionSteer(depth_mono8_img, depth_float_img);

}
  

double getYaw(geometry_msgs::Quaternion orientation) {
  double yaw = tf::getYaw(orientation);
  return yaw;
}


void callbackGt(const nav_msgs::Odometry& msg)
{
  // geometry_msgs::Vector3 pos = msg.pose.pose.position;
  // std::cout << "height: "<< msg.pose.pose.position.z << std::endl;
  if (msg.pose.pose.position.z > ADJUST_HEIGHT_MAX){
    adjust_height=-1;
  }else if (msg.pose.pose.position.z < ADJUST_HEIGHT_MIN){ // Was 0.5
    adjust_height=1;
  }else{
    adjust_height=0;
  }


  CURRENT_YAW = getYaw(msg.pose.pose.orientation) + CV_PI;
  cout << "Yaw: " << CURRENT_YAW << endl;

  if (state == TURN_GOAL) {
    if (abs(CURRENT_YAW - INITIAL_DIR) >  YAW_MARGIN ){
      if ((CURRENT_YAW - INITIAL_DIR) >= 0) {
        ADJUST_DIR = -1;
      } 
      else {
        ADJUST_DIR = 1;
      }
    }
    else{
      ADJUST_DIR = 0;
      state = OBSTACLE_AVOID;
    }
  }
  //std::cout << "twist message ang z: "<< ang.z << std::endl;
}

private: //private fields of callback
bool is_first_image_;
bool is_first_image_depth_;
bool has_camera_info_;
size_t count_;
string control;
};

geometry_msgs::Twist get_twist(){
  geometry_msgs::Twist twist;
  
  //FSM
  switch(state){
    case HOVER://hover but adjust height
      twist.linear.x = 0.0;//straight
      twist.linear.y = 0.0;
      twist.linear.z = adjust_height;//up

      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 0.0;
      if(FSM_COUNTER >= FSM_COUNTER_THRESH){
       state = 1;
	     //FSM_COUNTER_THRESH = rng.uniform(1000,1500);
       //FSM_COUNTER_THRESH = rng.uniform(oa_dur,oa_dur*2);
       FSM_COUNTER = 0;
	takeoff=true;
      }
      break;
    case OBSTACLE_AVOID://do obstacle avoidance
      //steer in horizontal plane
      if (STEER_DIR == 1)
      {
      	twist.linear.x = 0.8;//straight
      	twist.linear.y = 0.0;
      	twist.linear.z = adjust_height;//up

      	twist.angular.x = 0.0;
      	twist.angular.y = 0.0;
      	twist.angular.z = 0.0;
	
      }
      else if (STEER_DIR == 0)// turn left
      {
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = adjust_height;//up

      	twist.angular.x = 0.0;
      	twist.angular.y = 0.0;
      	twist.angular.z = STEER_SPEED_SCALE;//+1.0;//turn left
      }
      else if  (STEER_DIR == 2)//steer_dir == 2 turn right
      {
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = adjust_height;//up

      	twist.angular.x = 0.0;
      	twist.angular.y = 0.0;
      	twist.angular.z = -STEER_SPEED_SCALE;//-1.0;//turn right
      } 
      else {
      	twist.linear.x = 0.0;
      	twist.linear.y = 0.0;
      	twist.linear.z = adjust_height;//up

      	twist.angular.x = 0.0;
      	twist.angular.y = 0.0;
      	twist.angular.z = 0.0;
      }
      // if(FSM_COUNTER >= FSM_COUNTER_THRESH){
      // 	state = 2 ;
      // 	FSM_COUNTER_THRESH = rng.uniform(rnd_dur,rnd_dur*2);
      // 	counter = 0;
      // 	turn =-1+2*rng.uniform(0,2);
      // 	cout << "turn " << turn ;
      	
      // }
      break;
    case TURN_GOAL:
      twist.linear.x = 1.0; // was 0 [klaas edit 21/8/16]
      twist.linear.y = 0.0;
      twist.linear.z = adjust_height;//up

      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = ADJUST_DIR;
      break;
    case OBSTACLE_AVOID_CORNER:
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = adjust_height;//up

        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = turn*STEER_SPEED_SCALE;
        if(FSM_COUNTER >= FSM_COUNTER_THRESH){
         state = 1;
         // FSM_COUNTER_THRESH = rng.uniform(oa_dur,oa_dur*2);
         FSM_COUNTER = 0;
       }
      break;
    // case OBSTACLE_AVOID_VERT:
    //     twist.linear.x = 0.8;
    //     twist.linear.y = 0.0;
    //     twist.linear.z = VERTICLE_DIR;

    //     twist.angular.x = 0.0;
    //     twist.angular.y = 0.0;
    //     twist.angular.z = 0.0;
    //   break;
    case HEIGHT_ADJUST_UP_DOWN:
      // go up if counter < thresh1
      twist.linear.x = 0.0;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;

      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 0.0;
      cout << "UP DOWN " << endl;
      if (FSM_COUNTER_HEIGHT_ADJUST < FSM_COUNTER_THRESH_HEIGHT_ADJUST1) {
        twist.linear.z = 1;
      }
      // else go forw if counter < thresh2
      else if (FSM_COUNTER_HEIGHT_ADJUST < FSM_COUNTER_THRESH_HEIGHT_ADJUST2) {
        twist.linear.x = 0.8;
      }
      // else go down
      else {
        twist.linear.z = -1;
      }

      FSM_COUNTER_HEIGHT_ADJUST++;



      // counter == thresh3 --> obstacle_avoid

      if(FSM_COUNTER_HEIGHT_ADJUST >= FSM_COUNTER_THRESH_HEIGHT_ADJUST3){
       state = OBSTACLE_AVOID;
       // FSM_COUNTER_THRESH = rng.uniform(oa_dur,oa_dur*2);
       FSM_COUNTER_HEIGHT_ADJUST = 0;
       }
      break;

    case HEIGHT_ADJUST_DOWN_UP:
      twist.linear.x = 0.0;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;

      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 0.0;
      cout << "DOWN UP " << endl;
      // go down if counter < thresh1
      if (FSM_COUNTER_HEIGHT_ADJUST < FSM_COUNTER_THRESH_HEIGHT_ADJUST1) {
        twist.linear.z = -1;
      }
      // else go forw if counter < thresh2
      else if (FSM_COUNTER_HEIGHT_ADJUST < FSM_COUNTER_THRESH_HEIGHT_ADJUST2) {
        twist.linear.x = 0.8;
      }
      // else go up
      else {
        twist.linear.z = 1;
      }

      FSM_COUNTER_HEIGHT_ADJUST++;



      // counter == thresh3 --> obstacle_avoid

      if(FSM_COUNTER_HEIGHT_ADJUST >= FSM_COUNTER_THRESH_HEIGHT_ADJUST3){
       state = OBSTACLE_AVOID;
       // FSM_COUNTER_THRESH = rng.uniform(oa_dur,oa_dur*2);
       FSM_COUNTER_HEIGHT_ADJUST = 0;
       }
      break;
    }
    FSM_COUNTER=FSM_COUNTER+1;
    // cout << "state: " << state << ". next state: " << FSM_COUNTER << "/" << FSM_COUNTER_THRESH << endl;

    return twist;
 }

 int main(int argc, char** argv)
 {
  ros::init(argc, argv, "teleop_autopilot", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  //std::string topic = nh.resolveName("image");
  std::string topic_depth = "/ardrone/kinect/depth/image_raw";

  Callbacks callbacks;
  
  std::string height_behavior = nh.resolveName("height");
  cout << "height behavior" << height_behavior << endl;
  if(strcmp(height_behavior.c_str(), "/high") == 0){ //flying high
    //cout << "###FLY HIGH ###"<<endl;
    ADJUST_HEIGHT_MAX = 4; 
    ADJUST_HEIGHT_MIN = 2.5;
  }else{//flying low (default)
    //cout << "###FLY LOW ###"<<endl;
    ADJUST_HEIGHT_MAX = 1.5; 
    ADJUST_HEIGHT_MIN = 0.75;
  }
  // Useful when CameraInfo is not being published
   //depth
  image_transport::Subscriber sub_image_depth = it.subscribe(
    topic_depth, 1, boost::bind(&Callbacks::callbackWithoutCameraInfoWithDepth, &callbacks, _1));

  
  // Make subscriber to ground_truth in order to get the psotion.
  //ros::Subscriber subControl = nh.subscribe("/ground_truth/state/pose/pose/position",1,&Callbacks::callbackGt, &callbacks);
  ros::Subscriber subControl = nh.subscribe("/ground_truth/state",1,&Callbacks::callbackGt, &callbacks);
  
  // Make subscriber to cmd_vel in order to set the name.
  ros::Publisher pubControl = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  
  // Make Publisher to cmd_vel in order to set the velocity.
  ros::Publisher pubTakeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  
  ros::Rate loop_rate(20);

  debugPub = nh.advertise<std_msgs::Float32>("debug_autopilot", 1000);

  geometry_msgs::Twist twist;
  
  twist.linear.x = 0.5;//straight
  twist.linear.y = 0.0;//left
  twist.linear.z = adjust_height;//up
  
  twist.angular.x = 0.05;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
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
