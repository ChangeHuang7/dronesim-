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
#include <nav_msgs/Odometry.h>

#include <fstream>      // std::ifstream
#include <sstream>     // std::cout

#include <std_msgs/Float32.h>

#include "stdio.h"
#include <string>
#include <cmath>

using namespace std;

const double PI = 3.1415926535897;
const float STANDARD_RANGE = 2;
ros::Publisher pubGoalAngle;
string waypoint_filename;

struct Waypoint {
  float x,y,z;
  float range, dir;
  int id;
};

vector<Waypoint> waypointList;
int lastWaypoint = -1;

void callbackGt(const nav_msgs::Odometry& msg)
{
  // Check if drone is close to next waypoint
  float dx = waypointList.at(lastWaypoint + 1).x - msg.pose.pose.position.x;
  float dy = waypointList.at(lastWaypoint + 1).y - msg.pose.pose.position.y;
  float distance = sqrt(pow(dx, 2) + pow(dy, 2));
  cout << "Distance to next waypoint (" << waypointList.at(lastWaypoint + 1).x << ", " << waypointList.at(lastWaypoint + 1).y
    << "): " << distance << endl;
  if (distance < waypointList.at(lastWaypoint + 1).range) {
    // Publish message to controller
    std_msgs::Float32 msg;
    msg.data = waypointList.at(lastWaypoint + 1).dir;
    pubGoalAngle.publish(msg);
    // Increment waypoint counter
    lastWaypoint++;

    cout << "Reached " << lastWaypoint << ", now moving to " << lastWaypoint + 1 << endl;
  }

}

void initWaypoints() {
  // Read file
  ifstream file(waypoint_filename.c_str());
  // Save as waypoints
  float x_temp, y_temp, z_temp;
  int id;
  float direction;
  while (file >> id >> x_temp >> y_temp >> z_temp) {
    cout << "Waypoint " << id << ": (x,y,z) (" << x_temp << ", "  <<  y_temp << ", " << z_temp << ")" << endl;
    Waypoint newWaypoint;
    newWaypoint.x = x_temp; newWaypoint.y = y_temp; newWaypoint.z = z_temp;
    newWaypoint.id = id;
    newWaypoint.dir = 0;
    newWaypoint.range = STANDARD_RANGE;
    waypointList.push_back(newWaypoint);
  }

  // Calculate the goal directions
  for (unsigned int i = 0; i < waypointList.size() - 1; i++) {
    float dx = waypointList.at(i+1).x - waypointList.at(i).x;
    float dy = waypointList.at(i+1).y - waypointList.at(i).y;

    waypointList.at(i).dir = atan2(dy, dx) + PI;
    cout << "Direction waypoint " << i << ": " << waypointList.at(i).dir << endl;
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
  // Init publisher to controller
  pubGoalAngle = nh.advertise<std_msgs::Float32>("/autopilot/goal_angle",10);
  // Sleep to make sure the controller node is active before publishing first message
  ros::Duration(1).sleep();

  // Get waypoint filename
  if(!nh.getParam("waypoint_loc", waypoint_filename)) {
    cout << "No waypoint file" << endl;
    exit(-1);
  }
  cout << "Reading waypoint file: " << waypoint_filename << endl;

  ros::Rate loop_rate(10);
  initWaypoints();
  ros::spin();
} 
