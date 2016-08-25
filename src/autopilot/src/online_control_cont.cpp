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


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <fstream>      // std::ifstream
#include <sstream>     // std::cout

#include <std_msgs/Empty.h>
//#include <unistd.h>
//#include <std_srvs/Empty.h>
//#include <algorithm>

//#include "stdio.h"
//#include <string>

//#include <stdlib.h>
//#include <sys/stat.h>
//#include <boost/bind.hpp>

using namespace std;
bool takeoff = false;
int FSM_COUNTER_THRESH=100;//wait for some time before taking off
int FSM_COUNTER = 0;

float speed_scale = 0.5;//10/20; //if the frame rate at training is 30 and the framerate at testing is 3 the speed should be 10 time lower.

int control_state = 0; //define some states that override the general obstacle avoidance behavior: 0 ~ wait, 1 ~ do obstacle avoidance, 2 ~ turn for a random amount of time in 1 direction
int frameNumber = 0; //next frame we are waiting for.
string control_location = "/home/jay/data/control_output/";
int adjust_height = 0; //0 means no adjust, 1 means go up, -1 means go down

geometry_msgs::Twist twist;

void callbackGt(const nav_msgs::Odometry& msg)
{
  //geometry_msgs::Vector3 pos = msg.pose.pose.position;
  //std::cout << "height: "<< msg.pose.pose.position.z << std::endl;
  if (msg.pose.pose.position.z > 1.5){
    adjust_height=-1;
  }else if (msg.pose.pose.position.z < 0.3){
    adjust_height=1;
  }else{
    adjust_height=0;
  }
  //std::cout << "twist message ang z: "<< ang.z << std::endl;
}

geometry_msgs::Twist get_twist(){
  //take off after FSM counter greater than the threshold
  if(FSM_COUNTER > FSM_COUNTER_THRESH) takeoff=true;
  //control_states are indices for current control vector:
  //[hover, back, forward, turn right, turn left, down, up, rotate cw, rotate ccw]
  //FSM
 
  FSM_COUNTER=FSM_COUNTER+1;
  return twist;
}

//Read control out of file and update control_state
void read_control(){
  //Check folder for new control and set the state accordingly
  //only apply velocity in case of new control
  int control = -1;
  int i = frameNumber+1;
  stringstream name;
  name << control_location << i << ".txt";
  string name_str = name.str();
  ifstream file(name_str.c_str());

  float x_lin, y_lin, z_lin, x_ang, y_ang ,z_ang;
  if(file)
  {
    // File should have 6 characters seperated by a space, going from linear xyz to angular xyz
    file >> x_lin >> y_lin >> z_lin >> x_ang >> y_ang >> z_ang;
    cout << x_lin << " " <<  y_lin << " " <<  z_lin << " " <<  x_ang << " " <<  y_ang << " " <<  z_ang << endl;
    // Update twist if there was a file
    twist.linear.x = x_lin;
    twist.linear.y = y_lin;
    twist.linear.z = z_lin;
    twist.angular.x = x_ang;
    twist.angular.y = y_ang;
    twist.angular.z = z_ang;
    frameNumber++;
  }
  
}

int main(int argc, char** argv)
{
  stringstream command;
  command << "exec rm -r "+control_location+"/*"; 
  //empty control location
  system(command.str().c_str());
  ros::init(argc, argv, "autopilot", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  
  // Make subscriber to ground_truth in order to get the psotion.
  ros::Subscriber subControl = nh.subscribe("/ground_truth/state",1,callbackGt);
  
  // Make subscriber to cmd_vel in order to set the name.
  ros::Publisher pubControl = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // Make Publisher to takeoff in order to set the velocity.
  ros::Publisher pubTakeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  
  ros::Rate loop_rate(20);

  geometry_msgs::Twist twist;
  
  while(ros::ok()){
    read_control();
    //control_state = 0;
    twist = get_twist();
    //cout << twist << endl;
    pubControl.publish(twist);
    if(takeoff){
      std_msgs::Empty msg;
      pubTakeoff.publish(msg);
    }
    
    loop_rate.sleep();
    ros::spinOnce();
  }
} 
