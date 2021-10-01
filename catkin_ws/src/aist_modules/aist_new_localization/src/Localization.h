// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba
//
/*!
 *  \file	Localization.h
 *  \author	Toshio Ueshiba
 */
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <iostream>
#include <cstdint>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <aist_depth_filter/PlaneStamped.h>
#include <aist_new_localization/LocalizeAction.h>
#include <std_srvs/Trigger.h>
#include <opencv2/core.hpp>

namespace aist_new_localization
{
/************************************************************************
*  class Localization							*
************************************************************************/
class Localization
{
  private:
    using action_t	 = LocalizeAction;
    using feedback_t	 = LocalizeFeedback;
    using result_t	 = LocalizeResult;
    using goal_cp	 = LocalizeGoalConstPtr;
    using server_t	 = actionlib::SimpleActionServer<action_t>;

    using value_t	 = float;
    using vector3_t	 = cv::Vec<value_t, 3>;
    using plane_t	 = aist_depth_filter::PlaneStamped;

    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using image_t	 = sensor_msgs::Image;
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using image_p	 = sensor_msgs::ImagePtr;
    using cloud_t	 = sensor_msgs::PointCloud2;
    using sync_policy_t	 = message_filters::sync_policies::
				ApproximateTime<camera_info_t, image_t>;

    static constexpr uint32_t	CHECK_UPPER_BORDER	= 0x1;
    static constexpr uint32_t	CHECK_RIGHT_BORDER	= 0x2;
    static constexpr uint32_t	CHECK_LOWER_BORDER	= 0x4;
    static constexpr uint32_t	CHECK_LEFT_BORDER	= 0x8;

  public:
		Localization(const ros::NodeHandle& nh)			;

    void	run()							;

  private:
    void	goal_cb()						;
    void	preempt_cb()						;
    void	localize_cb(const camera_info_cp& camera_info,
			    const image_cp& depth)			;

    void	transform_plane(const std::string& target_frame,
				const plane_t& plane,
				vector3_t& normal,
				value_t& distance)		const	;
    static vector3_t
		view_vector(const camera_info_cp& camera_info,
			    value_t u, value_t v)			;
    tf::Transform
		refine_transform(const std::string& object_name,
				 const tf::Transform& Tcm,
				 const camera_info_cp& camera_info,
				 const image_cp& depth,
				 uint32_t check_borders,
				 value_t& error)		const	;
    template <class ITER>
    bool	within_view_volume(ITER begin, ITER end,
				   const camera_info_cp& camera_info,
				   uint32_t check_borders)	const	;

  private:
    ros::NodeHandle					_nh;

    const tf::TransformListener				_listener;

    message_filters::Subscriber<camera_info_t>		_camera_info_sub;
    message_filters::Subscriber<image_t>		_depth_sub;
    message_filters::Synchronizer<sync_policy_t>	_sync;

    const ros::Publisher				_model_cloud_pub;
    const ros::Publisher				_data_cloud_pub;

    server_t						_localize_srv;
    goal_cp						_current_goal;

    ddynamic_reconfigure::DDynamicReconfigure		_ddr;
    const std::string					_pcd_dir;
    int							_niterations;
    double						_max_distance;
    const double					_transformation_epsilon;
    const double					_fitness_epsilon;
};

}	// namespace aist_localization
#endif	// LOCALIZATION_H
