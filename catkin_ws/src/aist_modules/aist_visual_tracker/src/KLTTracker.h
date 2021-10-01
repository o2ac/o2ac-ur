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
 *  \file	KLTTracker.h
 *  \author	Toshio Ueshiba
 *  \brief	ROS node for tracking corners in 2D images
 */
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include "klt/klt.h"

namespace aist_visual_tracker
{
/************************************************************************
*  class KLTTracker							*
************************************************************************/
class KLTTracker
{
  private:
    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using image_t	 = sensor_msgs::Image;
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using context_t	 = KLT_TrackingContextRec;

  public:
		KLTTracker(const ros::NodeHandle& nh)			;
		~KLTTracker()						;

    void	run()							;

  private:
  // Service callbacks
    bool	select_cb(std_srvs::Trigger::Request&  req,
			  std_srvs::Trigger::Response& res)		;

  // ddynamic_reconfigure callbacks
    void	set_window_radius_cb(int context_t::* width,
				     int context_t::* height,
				     int radius)			;
    void	set_sequential_mode_cb(bool enable)			;
    template <class T>
    void	set_ctx_cb(T context_t::* field, T value)		;
    template <class T>
    void	set_param_cb(T KLTTracker::* field, T value,
			     bool select)				;

  // topic callbacks
    void	track_cb(const image_cp& image)				;

    void	track(const image_t& image)				;

    size_t	nfeatures()	const	{ return _featureTable->nFeatures; }
    size_t	nframes()	const	{ return _featureTable->nFrames; }
    KLT_Feature	operator [](size_t j)				const	;
    KLT_Feature	operator ()(size_t i, size_t j)			const	;
    void	selectGoodFeatures(const image_t& image)		;
    void	trackFeatures(const image_t& image)			;
    KLT_FeatureList
		featureList()		{ return _featureList; }

  private:
    ros::NodeHandle					_nh;

    const ros::ServiceServer				_select_srv;

    image_transport::ImageTransport			_it;
    image_transport::Subscriber				_image_sub;
    const image_transport::Publisher			_image_pub;

  // Tracker parameters and dynamic_reconfigure server for setting them
    ddynamic_reconfigure::DDynamicReconfigure		_ddr;
    int							_nfeatures;
    int							_nframes;
    bool						_replace;

  // Tracker stuffs
    KLT_TrackingContext					_ctx;
    KLT_FeatureList					_featureList;
    KLT_FeatureTable					_featureTable;
    bool						_select;
    size_t						_frame;
    int							_previous_width;
    int							_previous_height;
    int							_marker_size;
    int							_marker_thickness;
};

inline KLT_Feature
KLTTracker::operator [](size_t j) const
{
    return _featureTable->feature[j][_frame];
}

}	// namespace aist_visual_tracker
