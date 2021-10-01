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
  \file	 Calibrator.cpp
  \brief Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/
#include <fstream>
#include <sstream>
#include <ctime>
#include <cstdlib>	// for std::getenv()
#include <sys/stat.h>	// for mkdir()
#include <errno.h>
#include <tf/transform_datatypes.h>
#include <yaml-cpp/yaml.h>
#include "Calibrator.h"
#include "HandeyeCalibration.h"

//#define DEBUG

namespace aist_handeye_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
Calibrator::Calibrator(const ros::NodeHandle& nh)
    :_nh(nh),
     _get_parameters_srv(
	 _nh.advertiseService("get_parameters",
			      &Calibrator::get_parameters, this)),
     _get_sample_list_srv(
	 _nh.advertiseService("get_sample_list",
			      &Calibrator::get_sample_list, this)),
     _take_sample_srv(
	 _nh.advertiseService("take_sample", &Calibrator::take_sample, this)),
     _compute_calibration_srv(
	 _nh.advertiseService("compute_calibration",
			      &Calibrator::compute_calibration, this)),
     _save_calibration_srv(
	 _nh.advertiseService("save_calibration",
			      &Calibrator::save_calibration, this)),
     _reset_srv(_nh.advertiseService("reset", &Calibrator::reset, this)),
     _listener(),
     _use_dual_quaternion(false),
     _eye_on_hand(true),
     _timeout(5.0)
{
    get_parameters();

    ROS_INFO_STREAM("calibrator started");
}

Calibrator::~Calibrator()
{
}

void
Calibrator::run()
{
    ros::spin();
}

const std::string&
Calibrator::camera_frame() const
{
    return _eMc.child_frame_id;
}

const std::string&
Calibrator::effector_frame() const
{
    return _eMc.header.frame_id;
}

const std::string&
Calibrator::object_frame() const
{
    return _wMo.child_frame_id;
}

const std::string&
Calibrator::world_frame() const
{
    return _wMo.header.frame_id;
}

void
Calibrator::get_parameters()
{
    _use_dual_quaternion = _nh.param<bool>("use_dual_quaternion", false);
    _eye_on_hand	 = _nh.param<bool>("eye_on_hand", true);

    if (_eye_on_hand)
    {
	_eMc.header.frame_id = _nh.param<std::string>("robot_effector_frame",
						      "tool0");
	_wMo.header.frame_id = _nh.param<std::string>("robot_base_frame",
						      "base_link");
    }
    else
    {
	_wMo.header.frame_id = _nh.param<std::string>("robot_effector_frame",
						      "tool0");
	_eMc.header.frame_id = _nh.param<std::string>("robot_base_frame",
						      "base_link");
    }

    _eMc.child_frame_id = _nh.param<std::string>("camera_frame",
						 "camera_frame");
    _wMo.child_frame_id = _nh.param<std::string>("marker_frame",
						 "marker_frame");
}

bool
Calibrator::get_parameters(std_srvs::Trigger::Request&,
			   std_srvs::Trigger::Response& res)
{
    try
    {
	get_parameters();

	res.success = true;
	res.message = "succeeded.";

	ROS_INFO_STREAM("take_sample(): " << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	ROS_ERROR_STREAM("get_parameters(): " << res.message);
    }

    return res.success;
}

bool
Calibrator::get_sample_list(GetSampleList::Request&,
			    GetSampleList::Response& res)
{
    res.success = true;
    res.message = std::to_string(_cMo.size()) + " samples in hand.";
    res.cMo	= _cMo;
    res.wMe	= _wMe;

    ROS_INFO_STREAM("get_sample_list(): " << res.message);

    return true;
}

bool
Calibrator::take_sample(std_srvs::Trigger::Request&,
			std_srvs::Trigger::Response& res)
{
    try
    {
	ros::Time	time;
	std::string	error_string;
	_listener.getLatestCommonTime(camera_frame(), object_frame(), time,
				      &error_string);
	_listener.waitForTransform(world_frame(), effector_frame(),
				   time, ros::Duration(_timeout));

	tf::StampedTransform	cMo, wMe;
	_listener.lookupTransform(camera_frame(), object_frame(),   time, cMo);
	_listener.lookupTransform(world_frame(),  effector_frame(), time, wMe);

	transformMsg_t	msg;
	tf::transformStampedTFToMsg(cMo, msg);
	_cMo.emplace_back(msg);
	tf::transformStampedTFToMsg(wMe, msg);
	_wMe.emplace_back(msg);

	res.success = true;
	res.message = "succeeded.";

	ROS_INFO_STREAM("take_sample(): " << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	ROS_ERROR_STREAM("take_sample(): " << res.message);
    }

    return res.success;
}

bool
Calibrator::compute_calibration(ComputeCalibration::Request&,
				ComputeCalibration::Response& res)
{
    try
    {
	using transform_t	= TU::Transform<double>;

	ROS_INFO_STREAM("compute_calibration(): computing with "
			<< (_use_dual_quaternion ? "DUAL quaternion"
						 : "SINGLE quaternion")
			<< " algorithm...");

	std::vector<transform_t>	cMo, wMe;
	for (size_t i = 0; i < _cMo.size(); ++i)
	{
	    cMo.emplace_back(_cMo[i].transform);
	    wMe.emplace_back(_wMe[i].transform);
	}

	const auto	eMc = (_use_dual_quaternion ?
			       TU::cameraToEffectorDual(cMo, wMe) :
			       TU::cameraToEffectorSingle(cMo, wMe));
	const auto	wMo = TU::objectToWorld(cMo, wMe, eMc);

	const auto	now = ros::Time::now();
	_eMc.header.stamp = now;
	_eMc.transform	  = eMc;
	_wMo.header.stamp = now;
	_wMo.transform	  = wMo;

	std::ostringstream	sout;
	TU::evaluateAccuracy(sout, cMo, wMe, eMc, wMo);
	res.success = true;
	res.message = sout.str();
	res.eMc	    = _eMc;
	res.wMo	    = _wMo;

	ROS_INFO_STREAM("compute_calibration(): " << res.message);

#ifdef DEBUG
	std::ofstream	out("cMo_wMe_pairs.txt");
	out << cMo.size() << std::endl;
	for (size_t i = 0; i < cMo.size(); ++i)
	    out << cMo[i] << std::endl
		<< wMe[i] << std::endl << std::endl;
	out << sout.str();
#endif
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	ROS_ERROR_STREAM("compute_calibration(): " << res.message);
    }

    return res.success;
}

bool
Calibrator::save_calibration(std_srvs::Trigger::Request&,
			     std_srvs::Trigger::Response& res)
{
    try
    {
	YAML::Emitter	emitter;
	emitter << YAML::BeginMap;

	emitter << YAML::Key   << "parent"
		<< YAML::Value << effector_frame();
	emitter << YAML::Key   << "child"
		<< YAML::Value << camera_frame();
	emitter << YAML::Key   << "transform"
		<< YAML::Value
		<< YAML::Flow
		<< YAML::BeginMap
		<< YAML::Key   << "x"
		<< YAML::Value << _eMc.transform.translation.x
		<< YAML::Key   << "y"
		<< YAML::Value << _eMc.transform.translation.y
		<< YAML::Key   << "z"
		<< YAML::Value << _eMc.transform.translation.z
		<< YAML::Key   << "qx"
		<< YAML::Value << _eMc.transform.rotation.x
		<< YAML::Key   << "qy"
		<< YAML::Value << _eMc.transform.rotation.y
		<< YAML::Key   << "qz"
		<< YAML::Value << _eMc.transform.rotation.z
		<< YAML::Key   << "qw"
		<< YAML::Value << _eMc.transform.rotation.w
		<< YAML::EndMap;

	emitter << YAML::Key   << "marker_parent"
		<< YAML::Value << world_frame();
	emitter << YAML::Key   << "marker_child"
		<< YAML::Value << object_frame();
	emitter << YAML::Key   << "marker_transform"
		<< YAML::Value
		<< YAML::Flow
		<< YAML::BeginMap
		<< YAML::Key   << "x"
		<< YAML::Value << _wMo.transform.translation.x
		<< YAML::Key   << "y"
		<< YAML::Value << _wMo.transform.translation.y
		<< YAML::Key   << "z"
		<< YAML::Value << _wMo.transform.translation.z
		<< YAML::Key   << "qx"
		<< YAML::Value << _wMo.transform.rotation.x
		<< YAML::Key   << "qy"
		<< YAML::Value << _wMo.transform.rotation.y
		<< YAML::Key   << "qz"
		<< YAML::Value << _wMo.transform.rotation.z
		<< YAML::Key   << "qw"
		<< YAML::Value << _wMo.transform.rotation.w
		<< YAML::EndMap;

	const auto	tval = time(nullptr);
	const auto	tstr = ctime(&tval);
	tstr[strlen(tstr)-1] = '\0';
	emitter << YAML::Key   << "calibration_date"
		<< YAML::Value << tstr;

	emitter << YAML::EndMap;

      // Read calibration file name from parameter server.
	const auto calib_file
			= _nh.param<std::string>(
			    "calib_file",
			    getenv("HOME") +
			    std::string("/.ros/aist_handeye_calibration/calib.yaml"));

      // Open/create parent directory of the calibration file.
	const auto	dir = calib_file.substr(0,
						calib_file.find_last_of('/'));
	struct stat	buf;
	if (stat(dir.c_str(), &buf) && mkdir(dir.c_str(), S_IRWXU))
	    throw std::runtime_error("cannot create " + dir + ": "
						      + strerror(errno));

      // Open calibration file.
	std::ofstream	out(calib_file.c_str());
	if (!out)
	    throw std::runtime_error("cannot open " + calib_file + ": "
						    + strerror(errno));

      // Save calitration results.
	out << emitter.c_str() << std::endl;

	res.success = true;
	res.message = "saved in " + calib_file;
	ROS_INFO_STREAM("save_calibration(): " << res.message);
    }
    catch (const std::exception& err)
    {
	res.success = false;
	res.message = err.what();

	ROS_ERROR_STREAM("compute_calibration(): " << res.message);
    }

    return res.success;
}

bool
Calibrator::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    _cMo.clear();
    _wMe.clear();

    ROS_INFO_STREAM("reset(): all samples cleared.");

    return true;
}

}	// namespace aist_handeye_calibration
