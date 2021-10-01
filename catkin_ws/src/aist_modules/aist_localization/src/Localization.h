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
 *  \brief	ROS wrapper of Photoneo Localization SDK
 */
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <iostream>
#include <cstdint>
#include <memory>

#include <ros/ros.h>
#include <aist_depth_filter/FileInfo.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_localization/LocalizeAction.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <opencv2/core.hpp>

#include <PhoLocalization.h>

namespace aist_localization
{
/************************************************************************
*  class Localization							*
************************************************************************/
class Localization
{
  private:
    using localization_t = pho::sdk::PhoLocalization;

    using file_info_t	 = aist_depth_filter::FileInfo;
    using file_info_cp	 = aist_depth_filter::FileInfoConstPtr;
    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;

    using action_t	 = aist_localization::LocalizeAction;
    using feedback_t	 = aist_localization::LocalizeFeedback;
    using result_t	 = aist_localization::LocalizeResult;
    using goal_cp	 = aist_localization::LocalizeGoalConstPtr;
    using server_t	 = actionlib::SimpleActionServer<action_t>;

    using sync_policy_t  = message_filters::sync_policies::
				ApproximateTime<file_info_t, camera_info_t>;

    using value_t	 = float;
    using vector3_t	 = cv::Vec<value_t, 3>;

    struct setting_base
    {
	static std::string
			get_path(const std::string& name)
			{
			    auto	path = name;
			    std::replace(path.begin(), path.end(), '_', ' ');
			    return path;
			}

			setting_base(const std::string& name)
			    :path(get_path(name))			{}
	virtual		~setting_base()					{}

	virtual void	set_to(localization_t& localization)	const	= 0;
	virtual void	get_from(const localization_t& localization)	= 0;

	const std::string	path;
    };

  //! Setting parameter of type T for localization
    template <class T>
    struct setting : public setting_base
    {
      //! Initialize the variable with the current value set to the localizer.
			setting(const std::string& name,
				const localization_t& localization)
			    :setting_base(name), value()
			{
			    get_from(localization);
			}

      //! Set the value to the localizer.
	virtual void	set_to(localization_t& localization) const
			{
			    localization.setSetting(path, value);

			    ROS_DEBUG_STREAM('\"' << path
					     << "\" <-- " << value);
			}

      //! Get the value from the localizer.
	virtual void	get_from(const localization_t& localization)
			{
			    value = localization.getSetting<T>(path);

			    ROS_DEBUG_STREAM('\"' << path
					     << "\" --> " << value);
			}

	T		value;
    };

  public:
		Localization(const ros::NodeHandle& nh)			;

    void	run()						  const	;

  private:
    template <class T>
    void	register_variable(const std::string& name,
				  const std::string& description,
				  T min, T max)				;
    void	register_variable(const std::string& name,
				  const std::string& description,
				  std::initializer_list<
					  std::string> vals)		;
    void	preempt_cb()					  const	;
    void	sync_cb(const file_info_cp& file_info,
			const camera_info_cp& camera_info)		;
    void	localize_cb(const goal_cp& goal)			;
    void	localize_full(const goal_cp& goal)			;
    void	localize_in_plane(const goal_cp& goal)			;
    vector3_t	view_vector(value_t u, value_t v)		  const	;

  private:
    ros::NodeHandle					_nh;

    std::string						_ply_dir;
    std::unique_ptr<localization_t>			_localization;

    message_filters::Subscriber<file_info_t>		_file_info_sub;
    message_filters::Subscriber<camera_info_t>		_camera_info_sub;
    message_filters::Synchronizer<sync_policy_t>	_sync;
    file_info_cp					_file_info;
    camera_info_cp					_camera_info;

    server_t						_localize_srv;

    ddynamic_reconfigure::DDynamicReconfigure		_ddr;
    std::vector<std::unique_ptr<setting_base> >		_settings;
};

}	// namespace aist_localization
#endif	// LOCALIZATION_H
