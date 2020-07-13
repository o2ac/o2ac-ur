/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example of how to use rosparam_shorcuts
*/

// C++
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

int main(int argc, char** argv)
{
  std::string name = "example";
  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ROS_INFO_STREAM_NAMED(name, "Starting rosparam shortcuts example...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  double control_rate;
  int param1;
  std::size_t param2;
  ros::Duration param3;
  Eigen::Isometry3d param4;
  std::vector<double> param5;
  Eigen::Isometry3d param6;
  geometry_msgs::Pose param7;
  geometry_msgs::Pose param8;

  // Load rosparams
  ros::NodeHandle rpnh(nh, name);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name, rpnh, "control_rate", control_rate);  // Double param
  error += !rosparam_shortcuts::get(name, rpnh, "param1", param1);              // Int param
  error += !rosparam_shortcuts::get(name, rpnh, "param2", param2);              // SizeT param
  error += !rosparam_shortcuts::get(name, rpnh, "param3", param3);              // Duration param
  error += !rosparam_shortcuts::get(name, rpnh, "param4", param4);              // Isometry3d param
  error += !rosparam_shortcuts::get(name, rpnh, "param5", param5);              // std::vector<double>
  error += !rosparam_shortcuts::get(name, rpnh, "param6", param6);              // Isometry3d param
  error += !rosparam_shortcuts::get(name, rpnh, "param7", param7);              // geometry_msgs::Pose param
  error += !rosparam_shortcuts::get(name, rpnh, "param8", param8);              // geometry_msgs::Pose param
  // add more parameters here to load if desired
  rosparam_shortcuts::shutdownIfError(name, error);

  // Output values that were read in
  ROS_INFO_STREAM_NAMED(name, "control rate: " << control_rate);
  ROS_INFO_STREAM_NAMED(name, "param1: " << param1);
  ROS_INFO_STREAM_NAMED(name, "param2: " << param2);
  ROS_INFO_STREAM_NAMED(name, "param3: " << param3.toSec());
  ROS_INFO_STREAM_NAMED(name, "param4: Translation:\n" << param4.translation());
  ROS_INFO_STREAM_NAMED(name, "param5[0]: " << param5[0]);
  ROS_INFO_STREAM_NAMED(name, "param5[3]: " << param5[3]);
  ROS_INFO_STREAM_NAMED(name, "param6: Translation:\n" << param6.translation());
  ROS_INFO_STREAM_NAMED(name, "param7: Pose:\n" << param7);
  ROS_INFO_STREAM_NAMED(name, "param8: Pose:\n" << param8);
  ROS_INFO_STREAM_NAMED(name, "Shutting down.");
  ros::shutdown();

  return 0;
}
