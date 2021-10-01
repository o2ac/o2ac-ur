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
/*
 *  \file	MotionDetector.h
 *  \author	Toshio Ueshiba
 *  \brief	ROS node for tracking corners in 2D images
 */
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/bgsegm.hpp>
#include <aist_motion_detector/FindCabletipAction.h>
#include "Plane.h"

namespace aist_motion_detector
{
/************************************************************************
*  class MotionDetector							*
************************************************************************/
class MotionDetector
{
  private:
    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using image_t	 = sensor_msgs::Image;
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using image_p	 = sensor_msgs::ImagePtr;
    using sync_policy_t	 = message_filters::sync_policies::
				ApproximateTime<camera_info_t,
						image_t, image_t>;

    using bgsub_p	 = cv::Ptr<cv::BackgroundSubtractor>;

    using value_t	 = float;
    using point3_t	 = cv::Point3_<value_t>;
    using point2_t	 = cv::Point_<value_t>;
    using point_t	 = cv::Point;
    using vector3_t	 = cv::Vec<value_t, 3>;
    using line_t	 = TU::Plane<value_t, 2>;
    using plane_t	 = TU::Plane<value_t, 3>;

  public:
		MotionDetector(const ros::NodeHandle& nh)		;
		~MotionDetector()					;

    void	run()							;

  private:
  // action callbacks
    void	goal_cb()						;
    void	preempt_cb()						;

  // topic callbacks
    void	image_cb(const camera_info_cp& camera_info,
			 const image_cp& image, const image_cp& depth)	;

  // utility functions
    tf::Transform
		find_cabletip(cv::Mat& image, cv::Mat& depth,
			      const std::string& target_frame,
			      const camera_info_cp& camera_info) const	;
    static vector3_t
		view_vector(const camera_info_cp& camera_info,
			    value_t u, value_t v)			;

  private:
    ros::NodeHandle					_nh;

    image_transport::ImageTransport			_it;
    image_transport::SubscriberFilter			_image_sub;
    image_transport::SubscriberFilter			_depth_sub;
    message_filters::Subscriber<camera_info_t>		_camera_info_sub;
    message_filters::Synchronizer<sync_policy_t>	_sync;
    const image_transport::Publisher			_image_pub;
    const image_transport::CameraPublisher		_camera_pub;

    const tf::TransformListener				_listener;
    tf::TransformBroadcaster				_broadcaster;

    actionlib::SimpleActionServer<FindCabletipAction>	_find_cabletip_srv;
    FindCabletipGoalConstPtr				_current_goal;

  // Motion detector parameters and dynamic_reconfigure server for setting them
    ddynamic_reconfigure::DDynamicReconfigure		_ddr;

  // Motion detector stuffs
    bgsub_p						_bgsub;
    double						_search_top;
    double						_search_bottom;
    double						_search_width;
};

}	// namespace aist_motion_detector
