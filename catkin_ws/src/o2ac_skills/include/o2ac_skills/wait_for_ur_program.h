#pragma once

// License: BSD
// Author: Felix von Drigalski

#include "ros/ros.h"
#include "ur_dashboard_msgs/IsProgramRunning.h"
// Do not forget to add ur_robot_driver to your CMakeLists.txt

// Note: topic_namespace needs a leading slash.
bool waitForURProgram(std::string topic_namespace = "", ros::Duration timeout = ros::Duration(60.0))
{
  ROS_DEBUG("Waiting for UR program to finish. Only run this after sending custom URScripts and not the regular motion "
            "commands, or this call will not terminate before the timeout.");
  ros::NodeHandle n_;
  ros::ServiceClient srvclient = n_.serviceClient<ur_dashboard_msgs::IsProgramRunning>(topic_namespace + "/ur_hardware_interface/dashboard/program_running");
  ur_dashboard_msgs::IsProgramRunning srv;
  ros::Time t_start = ros::Time::now();
  ros::Duration time_passed = ros::Time::now() - t_start;
  ros::Duration(.2).sleep();
  while (true)
  {
    if (srvclient.call(srv))
    {
      if (!srv.response.program_running)
      {
        ROS_INFO("waitForURProgram has terminated in time.");
        return true;
      }
    }
    else
    {
      ROS_ERROR_STREAM("waitForURProgram polled an incorrect service address: " << topic_namespace << "ur_hardware_interface/dashboard/program_running" );
    }
    
    time_passed = ros::Time::now() - t_start;
    if (time_passed > timeout)
    {
      ROS_WARN("waitForURProgram was called but UR Program has not terminated within timeout.");
      return false;
    }

    ros::Duration(.05).sleep();
  }
}
