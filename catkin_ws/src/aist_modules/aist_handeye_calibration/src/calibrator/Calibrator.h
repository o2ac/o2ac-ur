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
  \file		Calibrator.h
  \brief	Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/

#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <aist_handeye_calibration/GetSampleList.h>
#include <aist_handeye_calibration/ComputeCalibration.h>

namespace aist_handeye_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator
{
  public:
    using transformMsg_t	= geometry_msgs::TransformStamped;

  public:
		Calibrator(const ros::NodeHandle& nh)			;
		~Calibrator()						;

    void	run()							;

  private:
    const std::string&	camera_frame()				const	;
    const std::string&	effector_frame()			const	;
    const std::string&	object_frame()				const	;
    const std::string&	world_frame()				const	;

    void	get_parameters()					;
    bool	get_parameters(std_srvs::Trigger::Request&,
			       std_srvs::Trigger::Response& res)	;
    bool	get_sample_list(GetSampleList::Request&,
				GetSampleList::Response& res)		;
    bool	take_sample(std_srvs::Trigger::Request&,
			    std_srvs::Trigger::Response& res)		;
    bool	compute_calibration(ComputeCalibration::Request&,
				    ComputeCalibration::Response& res)	;
    bool	save_calibration(std_srvs::Trigger::Request&,
				 std_srvs::Trigger::Response& res)	;
    bool	reset(std_srvs::Empty::Request&,
		      std_srvs::Empty::Response&)			;

  private:
    ros::NodeHandle		_nh;

    const ros::ServiceServer	_get_parameters_srv;
    const ros::ServiceServer	_get_sample_list_srv;
    const ros::ServiceServer	_take_sample_srv;
    const ros::ServiceServer	_compute_calibration_srv;
    const ros::ServiceServer	_save_calibration_srv;
    const ros::ServiceServer	_reset_srv;

    const tf::TransformListener	_listener;

    std::vector<transformMsg_t>	_cMo;	//!< in:  camera <- object   transform
    std::vector<transformMsg_t>	_wMe;	//!< in:  world  <- effector transform
    transformMsg_t		_eMc;	//!< out: effector <- camera transform
    transformMsg_t		_wMo;	//!< out: world    <- object transform

    bool			_use_dual_quaternion;

    bool			_eye_on_hand;
    double			_timeout;
};
}	// namespace aist_hnadeye_calibration
#endif	// !CALIBRATOR_H
