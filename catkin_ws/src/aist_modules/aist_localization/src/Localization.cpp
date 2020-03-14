/*!
 *  \file	Localization.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS wrapper of Photoneo Localization SDK
 */
#include <ros/package.h>
#include <tf/tf.h>
#include "Localization.h"

namespace aist_localization
{
/************************************************************************
*  class Localization							*
************************************************************************/
Localization::Localization(const std::string& name)
    :_nh(name),
     _camera_frame("world"),
     _plcf_dir(""),
     _localization(new pho::sdk::PhoLocalization()),
     _scene(),
     _scene_is_valid(false),
     _file_info_sub(_nh.subscribe<file_info_t>(
			"/file_info", 1, &Localization::file_info_cb, this)),
     _localize_srv(_nh, "localize",
		   boost::bind(&Localization::localize_cb, this, _1), false)
{
    _localize_srv.registerPreemptCallback(
	boost::bind(&Localization::preempt_cb, this));
    _localize_srv.start();

    _nh.param("camera_frame", _camera_frame, std::string("world"));
    _nh.param("plcf_dir", _plcf_dir,
	      ros::package::getPath("aist_localization") + "/plcf");
}

void
Localization::run() const
{
    ros::spin();
}

void
Localization::file_info_cb(const file_info_cp& file_info)
{
    try
    {
	ROS_INFO_STREAM("(Localization) Set frame to [" << file_info->frame
			<< "] and loading scene["
			<< file_info->file_path << "]..,");

	_camera_frame = file_info->frame;
	_scene = pho::sdk::SceneSource::File(file_info->file_path);
	_localization->SetSceneSource(_scene);
	_scene_is_valid = true;

	ROS_INFO_STREAM("(Localization)  succeeded.");
	return true;
    }
    catch (const std::exception& err)
    {
	_scene_is_valid = false;

	ROS_ERROR_STREAM("(Localization)  failed: " << err.what());
	return false;
    }
}

void
Localization::localize_cb(const goal_cp& goal) const
{
    try
    {
	ROS_INFO_STREAM("(Localization) Localizing object["
			<< goal->object_name << "].");

	if (!_scene_is_valid)
	    throw std::runtime_error("Scene is not set.");

      // Load configuration.
	const auto	plcf_file = _plcf_dir + '/'
				  + goal->object_name + ".plcf";
	if (!_localization->LoadLocalizationConfiguration(plcf_file))
	    throw std::runtime_error("  Failed to load configuration["
				     + plcf_file + "].");

      // Set stop criteria.
	_localization->SetStopCriterion(
	    pho::sdk::StopCriterion::NumberOfResults(goal->number_of_poses));

      // Execute localization.
	const auto	now = ros::Time::now();
	for (auto queue = _localization->StartAsync();
	     queue.Size() != goal->number_of_poses; )
	{
	    if (_localize_srv.isPreemptRequested())
	    {
		_localization->StopAsync();
		ROS_INFO_STREAM("(Localization)  Preempted.");
		return;
	    }

	    pho::sdk::LocalizationPose	locPose;
	    if (queue.GetNext(locPose, 500))	// Timeout = 500ms
	    {
		const auto object_frame = goal->object_name + '_'
					+ std::to_string(queue.Size() - 1);
		publish_feedback(locPose, now, object_frame);

		ROS_INFO_STREAM("(Localization)  Found "
				<< queue.Size() << "-th pose.");
	    }
	}

	_localization->StopAsync();
	ROS_INFO_STREAM("(Localization)  Completed.");

	result_t	result;
	result.success = true;
	_localize_srv.setSucceeded(result);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(Localization) " << err.what());

	_localization->StopAsync();

	result_t	result;
	result.success = false;
	_localize_srv.setAborted(result);
    }
}

void
Localization::preempt_cb() const
{
    _localize_srv.setPreempted();
}

void
Localization::publish_feedback(const pho::sdk::LocalizationPose& locPose,
			       ros::Time time,
			       const std::string& object_frame) const
{
    const auto&		mat = locPose.Transformation;
    const tf::Transform	transform(tf::Matrix3x3(
				      mat[0][0], mat[0][1], mat[0][2],
				      mat[1][0], mat[1][1], mat[1][2],
				      mat[2][0], mat[2][1], mat[2][2]),
				  tf::Vector3(0.001*mat[0][3],
					      0.001*mat[1][3],
					      0.001*mat[2][3]));
    feedback_t		feedback;
    feedback.pose.header.stamp	  = time;
    feedback.pose.header.frame_id = _camera_frame;
    tf::poseTFToMsg(transform, feedback.pose.pose);
    feedback.overlap = locPose.VisibleOverlap;
    _localize_srv.publishFeedback(feedback);
}

}	// namespace aist_localization
