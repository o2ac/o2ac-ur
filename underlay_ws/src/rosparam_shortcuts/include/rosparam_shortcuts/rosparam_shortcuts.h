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

#ifndef ROSPARAM_SHORTCUTS_ROSPARAM_SHORTCUTS_H
#define ROSPARAM_SHORTCUTS_ROSPARAM_SHORTCUTS_H

// C++
#include <string>
#include <vector>
#include <map>

// ROS
#include <ros/ros.h>

// Eigen
#include <Eigen/Geometry>

// this package
#include <rosparam_shortcuts/deprecation.h>

// geometry_msgs/Pose
#include <geometry_msgs/Pose.h>

namespace rosparam_shortcuts
{
// -------------------------------------------------------------------------------------------------
// Helper Functions
// -------------------------------------------------------------------------------------------------

/**
 * \brief Get a paremeter from the ROS param server. Note that does not provide for default values
 * \param parent_name - the name of the class that is calling this function, used for filtering out logging output by
 * namespacing it
 * \param nh - a ROS node handle
 * \param param_name - name of parameter to get
 * \param value - resulting loaded values, or no change if error (function returns false)
 * \return true on success
 */
bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, bool &value);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &params_name,
         std::map<std::string, bool> &parameters);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, double &value);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name,
         std::vector<double> &values);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, int &value);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, std::size_t &value);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name, std::string &value);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name,
         std::vector<std::string> &values);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name,
         ros::Duration &value);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name,
         Eigen::Isometry3d &value);

bool get(const std::string &parent_name, const ros::NodeHandle &nh, const std::string &param_name,
         geometry_msgs::Pose &value);

/**
 * \brief Output a string of values from an array for debugging
 * \param array of values
 * \return string of numbers separated by commas
 */
std::string getDebugArrayString(std::vector<double> values);

std::string getDebugArrayString(std::vector<std::string> values);

/**
 * \brief Convert from 6 doubles of [x,y,z] [r,p,y] or 7 doubles of [x, y, z, qw, qx, qy, qz] to a transform
 * \return true on success
 */
bool convertDoublesToEigen(const std::string &parent_name, std::vector<double> values, Eigen::Isometry3d &transform);

/**
 * \brief Check that there were no errors, and if there were, shutdown
 * \param error - total number of errors found
 */
void shutdownIfError(const std::string &parent_name, std::size_t error_count);

}  // namespace rosparam_shortcuts

#endif  // ROSPARAM_SHORTCUTS_ROSPARAM_SHORTCUTS_H
