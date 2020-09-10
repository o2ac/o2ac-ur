#pragma once

#include "ros/ros.h"
#include "ur_msgs/RobotModeDataMsg.h"
#include "std_msgs/Bool.h"
// Do not forget to add ur_robot_driver to your CMakeLists.txt

// Note: topic_namespace needs a leading slash.
bool isProgramRunning(std::string topic_namespace = "")
{
  ur_msgs::RobotModeDataMsg robot_mode_state;
  auto msg = ros::topic::waitForMessage<std_msgs::Bool>(topic_namespace + "/ur_hardware_interface/robot_program_running",
                                                                   ros::Duration(3));
  if (msg)
    return msg->data;
  else
  {
    ROS_ERROR("No message received from the robot. Is everything running? Is the namespace entered correctly with a "
              "leading slash?");
    ROS_ERROR_STREAM(topic_namespace << "/ur_hardware_interface/robot_program_running");
    ros::Exception e("No message received from the robot.");
    throw(e);
  }
}

// Note: topic_namespace needs a leading slash.
bool waitForURProgram(std::string topic_namespace = "", ros::Duration timeout = ros::Duration(60.0))
{
  ROS_DEBUG("Waiting for UR program to finish. Only run this after sending custom URScripts and not the regular motion "
            "commands, or this call will not terminate before the timeout.");
  ros::Time t_start = ros::Time::now();
  ros::Duration time_passed = ros::Time::now() - t_start;
  while (isProgramRunning(topic_namespace) && (time_passed < timeout))
  {
    ros::Duration(.05).sleep();
    time_passed = ros::Time::now() - t_start;
  }
  ROS_DEBUG("UR Program has terminated.");
  return true;
}