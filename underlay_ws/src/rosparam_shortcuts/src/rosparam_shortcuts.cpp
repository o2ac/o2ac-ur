/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Helpers for loading parameters from the parameter server
*/

// C++
#include <string>
#include <vector>
#include <map>

// this package
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Eigen/TF conversion
#include <eigen_conversions/eigen_msg.h>

namespace rosparam_shortcuts
{
bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, bool &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << nh.getNamespace() << "/" << param_name << "'.");
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << param_name << "' with value "
                                                           << value);

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &params_namespace,
         std::map<std::string, bool> &parameters)
{
  // Load a param
  if (!nh.hasParam(params_namespace))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameters in namespace '" << nh.getNamespace() << "/"
                                                                            << params_namespace << "'.");
    return false;
  }
  nh.getParam(params_namespace, parameters);

  // Debug
  for (std::map<std::string, bool>::const_iterator param_it = parameters.begin(); param_it != parameters.end();
       param_it++)
  {
    ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << params_namespace << "/"
                                                             << param_it->first << "' with value " << param_it->second);
  }

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, double &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << nh.getNamespace() << "/" << param_name << "'.");
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << param_name << "' with value "
                                                           << value);

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name,
         std::vector<double> &values)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << nh.getNamespace() << "/" << param_name << "'.");
    return false;
  }
  nh.getParam(param_name, values);

  if (values.empty())
    ROS_WARN_STREAM_NAMED(parent_name, "Empty vector for parameter '" << nh.getNamespace() << "/" << param_name << "'"
                                                                                                                   ".");

  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << param_name
                                                           << "' with values [" << getDebugArrayString(values) << "]");

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, int &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << nh.getNamespace() << "/" << param_name << "'.");
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << param_name << "' with value "
                                                           << value);

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, std::size_t &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << nh.getNamespace() << "/" << param_name << "'.");
    return false;
  }
  int nonsigned_value;
  nh.getParam(param_name, nonsigned_value);
  value = nonsigned_value;
  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << param_name << "' with value "
                                                           << value);

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, std::string &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << nh.getNamespace() << "/" << param_name << "'.");
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << param_name << "' with value "
                                                           << value);

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name,
         std::vector<std::string> &values)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << nh.getNamespace() << "/" << param_name << "'.");
    return false;
  }
  nh.getParam(param_name, values);

  if (values.empty())
    ROS_WARN_STREAM_NAMED(parent_name, "Empty vector for parameter '" << nh.getNamespace() << "/" << param_name << "'"
                                                                                                                   ".");

  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << param_name << "' with value "
                                                           << getDebugArrayString(values));

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, ros::Duration &value)
{
  double temp_value;
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << nh.getNamespace() << "/" << param_name << "'.");
    return false;
  }
  nh.getParam(param_name, temp_value);
  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << param_name << "' with value "
                                                           << value);

  // Convert to ros::Duration
  value = ros::Duration(temp_value);

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name,
         Eigen::Isometry3d &value)
{
  std::vector<double> values;

  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << nh.getNamespace() << "/" << param_name << "'.");
    return false;
  }
  nh.getParam(param_name, values);

  if (values.empty())
    ROS_WARN_STREAM_NAMED(parent_name, "Empty vector for parameter '" << nh.getNamespace() << "/" << param_name << "'"
                                                                                                                   ".");

  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << nh.getNamespace() << "/" << param_name
                                                           << "' with values [" << getDebugArrayString(values) << "]");

  // Convert to Eigen::Isometry3d
  convertDoublesToEigen(parent_name, values, value);

  return true;
}

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name,
         geometry_msgs::Pose &value)
{
  Eigen::Isometry3d eigen_pose;
  if (!get(parent_name, nh, param_name, eigen_pose))
    return false;

  tf::poseEigenToMsg(eigen_pose, value);
  return true;
}

std::string getDebugArrayString(std::vector<double> values)
{
  std::stringstream debug_values;
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    debug_values << values[i] << ",";
  }
  return debug_values.str();
}

std::string getDebugArrayString(std::vector<std::string> values)
{
  std::stringstream debug_values;
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    debug_values << values[i] << ",";
  }
  return debug_values.str();
}

bool convertDoublesToEigen(const std::string &parent_name, std::vector<double> values, Eigen::Isometry3d &transform)
{
  if (values.size() == 6)
  {
    // This version is correct RPY
    Eigen::AngleAxisd roll_angle(values[3], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(values[4], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(values[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> quaternion = roll_angle * pitch_angle * yaw_angle;

    transform = Eigen::Translation3d(values[0], values[1], values[2]) * quaternion;

    return true;
  }
  else if (values.size() == 7)
  {
    // Quaternion
    transform = Eigen::Translation3d(values[0], values[1], values[2]) * 
                Eigen::Quaterniond(values[3], values[4], values[5], values[6]);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Invalid number of doubles provided for transform, size=" << values.size());
    return false;
  }
}

void shutdownIfError(const std::string &parent_name, std::size_t error_count)
{
  if (!error_count)
    return;

  ROS_ERROR_STREAM_NAMED(parent_name, "Missing " << error_count << " ros parameters that are required. Shutting down "
                                                                   "to prevent undefined behaviors");
  ros::shutdown();
  exit(0);
}

}  // namespace rosparam_shortcuts
