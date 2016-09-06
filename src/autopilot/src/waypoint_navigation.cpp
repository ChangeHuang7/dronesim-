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




#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <fstream>      // std::ifstream
#include <sstream>     // std::cout

#include <std_msgs/Empty.h>
#include <unistd.h>
#include <std_srvs/Empty.h>
#include <algorithm>

#include "stdio.h"
#include <string>

#include <stdlib.h>
#include <sys/stat.h>
#include <boost/bind.hpp>

using namespace std;

string waypoint_filename;

struct Waypoint {
  float x,y,z;
  float range, dir;
  int id;
};

vector<Waypoint> waypointList;

void callbackGt(const nav_msgs::Odometry& msg)
{
  
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

void initWaypoints() {
  // Read file
  ifstream file(waypoint_filename.c_str());
  // Save as waypoints
  float x_temp, y_temp, z_temp;
  int id;
  float direction;
  while (file >> x_temp >> y_temp >> z_temp >> id >> direction) {
    cout << "Waypoint " << id << ": (x,y,z) (" << x_temp << ", "  <<  y_temp << ", "  z_temp << "), Direction: " << direction << endl;
    Waypoint newWaypoint;
    newWaypoint.x = x_temp; newWaypoint.y = y_temp; newWaypoint.z = z_temp;
    newWaypoint.id = id;
    newWaypoint.dir = direction;
  }
}

int main(int argc, char** argv)
{
  stringstream command;
  //command << "exec rm -r "+control_location+"/*"; 
  //empty control location
  system(command.str().c_str());
  ros::init(argc, argv, "autopilot", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  
  // Make subscriber to ground_truth in order to get the psotion.
  ros::Subscriber subControl = nh.subscribe("/ground_truth/state",1,callbackGt);
  
  ros::Rate loop_rate(10);

  geometry_msgs::Twist twist;

  int initial_file = 0;
  nh.getParam("last_control_output", initial_file);
  frameNumber = initial_file;
  
  while(ros::ok()){

    
    loop_rate.sleep();
    ros::spinOnce();
  }
} 
