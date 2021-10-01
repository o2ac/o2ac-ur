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
 *  \file	Localization.cpp
 *  \author	Toshio Ueshiba
 *  \brief	ROS wrapper of Photoneo Localization SDK
 */
#include <ros/package.h>
#include <tf/tf.h>
#include <opencv2/imgproc.hpp>
#include "Localization.h"

namespace aist_localization
{
/************************************************************************
*  class Localization							*
************************************************************************/
Localization::Localization(const ros::NodeHandle& nh)
    :_nh(nh),
     _ply_dir(""),
     _localization(new localization_t()),
     _file_info_sub(_nh, "/file_info",  1),
     _camera_info_sub(_nh, "/camera_info",  1),
     _sync(sync_policy_t(10), _file_info_sub, _camera_info_sub),
     _localize_srv(_nh, "localize",
		   boost::bind(&Localization::localize_cb, this, _1), false),
     _ddr(_nh),
     _settings()
{
  // Setup Localize action server.
    _localize_srv.registerPreemptCallback(boost::bind(&preempt_cb, this));
    _localize_srv.start();

    _nh.param("ply_dir", _ply_dir,
	      ros::package::getPath("o2ac_parts_description") + "/meshes");

    _sync.registerCallback(&Localization::sync_cb, this);

  // Setup dynamic reconfigure parameters for localization.
  //   0: Preprocessing and overall search strategy
    register_variable("Scene_Noise_Reduction",
    		      "Reduce scene noise",
    		      false, true);
    register_variable("Smart_Memory",
    		      "Remember the object poses found in the last run and uses them in the next search.",
    		      false, true);

  //   1: Segmentation
    register_variable("Scene_Clustering_Level",
		      "Clustering level in segmentation",
		      {"Low", "Normal", "High", "Very high"});
    register_variable("Scene_Minimal_Cluster_Size",
		      "Minimum number of points in a segment",
		      100, 4000000);
    register_variable("Scene_Maximal_Cluster_Size",
		      "Maximum number of points in a segment",
		      100, 4000000);

  //   2: Matching algorithm and parameters for search.
    register_variable("Matching_Algorithm",
		      "Surfaces: definite depths and few definite edges, Edges: definete edges, Combined: not rounded and definite edges",
		      {"Surfaces", "Edges", "Combined"});
    register_variable("Model_Keypoints_Sampling",
		       "Density of model subsampling. Should be dense when using \"Combined\" matching algorithm.",
		       {"Sparse", "Medium", "Dense"});
    register_variable("Local_Search_Radius",
		      "Radius for searching around keypoints",
		      {"Short", "Normal", "Medium"});

  //   3: Tolerance of matching to noise and accuracy
    register_variable("Feature_fit_consideration_level",
		      "Minimum percentage of the object segement size to the view",
		      0, 100);
    register_variable("Global_maximal_feature_fit_overflow",
		      "Percentage of all segments allowed to overflow the sum of segment sizes",
		      0, 100);

  //   4: Fine alignment
    register_variable("Fine_Alignment_Iterations",
		      "The number of iterations of fine alignment",
		      1, 100);
    register_variable("Fine_Alignment_Point_Set",
		      "Surafece: suitable for rounded objects, Edge: suitable for planar objects",
		      {"Surface", "Edges"});
    register_variable("Fine_Alignment_Point_Set_Sampling",
		      "Sampled: suitable when \"Fine Alignment Point Set\" is set to \"Surface\", Complete: suitable when \"Fine Alignment Point Set\" is set to \"Edge\"",
		      {"Sampled", "Complete"});

  //   5: Projection parameters
    register_variable("Projection_Tolerance",
		      "Allowed minimum percentage of visible part",
		      0, 100);
    register_variable("Projection_Hidden_Part_Tolerance",
		      "Allowed maximum percentage of occluded part",
		      0, 100);

  //   6: Overlap
    register_variable("Overlap",
		      "Minimum coverage of the matched points to the CAD model",
		      0.0, 100.0);

  // Pulish ddynamic_reconfigure service.
    _ddr.publishServicesTopics();
}

void
Localization::run() const
{
    ros::spin();
}

template <class T> void
Localization::register_variable(const std::string& name,
				const std::string& description, T min, T max)
{
  // Initialize the setting in the dynamic_reconfigure server
  // with the current value set to the localizer.
    const auto	p = new setting<T>(name, *_localization);
    _settings.emplace_back(p);
    _ddr.registerVariable<T>(name, &p->value, description, min, max);
}

void
Localization::register_variable(const std::string& name,
				const std::string& description,
				std::initializer_list<std::string> vals)
{
    std::map<std::string, std::string>	enum_dict;
    for (const auto& val : vals)
	enum_dict[val] = val;

  // Initialize the setting in the dynamic_reconfigure server
  // with the current value set to the localizer.
    const auto	p = new setting<std::string>(name, *_localization);
    _settings.emplace_back(p);
    _ddr.registerEnumVariable<std::string>(name, &p->value,
					   description, enum_dict);
}

void
Localization::preempt_cb() const
{
    _localization->StopAsync();
    _localize_srv.setPreempted();
    ROS_INFO_STREAM("(Localization)   *preempted*");
}

void
Localization::sync_cb(const file_info_cp& file_info,
		      const camera_info_cp& camera_info)
{
    _file_info   = file_info;
    _camera_info = camera_info;
}

void
Localization::localize_cb(const goal_cp& goal)
{
    if (!_file_info || !_file_info->plane_detected)
    {
	_localize_srv.setAborted();
	ROS_ERROR_STREAM("(Localization) Aborted a goal because no plane detected");
	return;
    }

    try
    {
	if (goal->in_plane)
	    localize_in_plane(goal);
	else
	    localize_full(goal);

	ROS_INFO_STREAM("(Localization)   completed.");

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
Localization::localize_full(const goal_cp& goal)
{
    ROS_INFO_STREAM("(Localization) Full localization of object["
		    << goal->object_name << "]...");

  // Set scene source.
    _localization->SetSceneSource(pho::sdk::SceneSource::File(
				      _file_info->file_path));
    ROS_DEBUG_STREAM("(Localization)   scene source succesfully set.");

  // Set model.
    _localization->RemoveModel();
    _localization->AddModel(pho::sdk::ModelOfObject::LoadModelFromFile(
				_ply_dir + '/' +
				goal->object_name + ".ply"));

  // Set settings in dynamic_reconfigure server to the localizer.
    for (const auto& setting : _settings)
	setting->set_to(*_localization);
    ROS_DEBUG_STREAM("(Localization)   settings succesfully set.");

  // Set stop criteria.
    _localization->AddStopCriterion(
	pho::sdk::StopCriterion::NumberOfResults(goal->nposes));
    ROS_DEBUG_STREAM("(Localization)   stop citeria succesfully set.");

  // Execute localization.
    auto	queue = _localization->StartAsync();
    for (pho::sdk::LocalizationPose locPose; queue.GetNext(locPose); )
    {
	const auto	k   = 0.001;	// convert milimeters to meters
	const auto&	mat = locPose.Transformation;
	const tf::Transform
		Tcp(tf::Matrix3x3(mat[0][0], mat[0][1], mat[0][2],
				  mat[1][0], mat[1][1], mat[1][2],
				  mat[2][0], mat[2][1], mat[2][2]),
		    tf::Vector3(k*mat[0][3], k*mat[1][3], k*mat[2][3]));

	feedback_t	feedback;
	feedback.pose.header = _file_info->header;
	tf::poseTFToMsg(Tcp, feedback.pose.pose);
	feedback.overlap = locPose.VisibleOverlap;
	_localize_srv.publishFeedback(feedback);

	ROS_INFO_STREAM("(Localization)   found "
			<< queue.Size() << "-th pose.");
    }
}

void
Localization::localize_in_plane(const goal_cp& goal)
{
    ROS_INFO_STREAM("(Localization) In-plane locaization of object["
		    << goal->object_name << "]...");

    if (!_file_info->plane_detected)
	throw std::runtime_error("  Dominant plane is not set.");

    const vector3_t	n(_file_info->normal.x,
			  _file_info->normal.y,
			  _file_info->normal.z);
    const value_t	d = _file_info->distance;

    size_t		i = 0;
    for (const auto& pose2d : goal->poses2d)
    {
	const auto	v = view_vector((pose2d.x > 0.0 ? pose2d.x :
					 0.5*_camera_info->width),
					(pose2d.y > 0.0 ? pose2d.y :
					 0.5*_camera_info->height));
	const auto	x = (-d/n.dot(v)) * v;
	auto		r = n.cross(vector3_t(std::cos(pose2d.theta),
					      std::sin(pose2d.theta), 0));
	r *= (value_t(1)/norm(r));
	const auto	q = r.cross(n);

      // Transformation from gaze point to camera
	const tf::Transform	transform(tf::Matrix3x3(q(0), r(0), n(0),
							q(1), r(1), n(1),
							q(2), r(2), n(2)),
					  tf::Vector3(x(0), x(1), x(2)));

	if (goal->sideways)
	    transform *= tf::Transform(tf::Matrix3x3(1.0, 0.0, 0.0,
						     0.0, 1.0, 0.0,
						     0.0, 0.0, 1.0),
				       tf::Vector3(goal->x_offset,
						   0.0,
						   goal->z_offset));
	else
	    transform *= tf::Transform(tf::Matrix3x3(0.0, 1.0, 0.0,
						     0.0, 0.0, 1.0,
						     1.0, 0.0, 0.0),
				       tf::Vector3(goal->x_offset,
						   0.0,
						   goal->z_offset));

	feedback_t	feedback;
	feedback.pose.header = _file_info->header;
	tf::poseTFToMsg(transform, feedback.pose.pose);
	feedback.overlap = 1.0;
	_localize_srv.publishFeedback(feedback);

	ros::Duration(0.1).sleep();	// for the client not to drop feedback

	ROS_INFO_STREAM("(Localization)   found " << ++i << "-th pose.");
    }
}

Localization::vector3_t
Localization::view_vector(value_t u, value_t v) const
{
    using point2_t	= cv::Point_<value_t>;

    cv::Mat_<value_t>	K(3, 3);
    std::copy_n(std::begin(_camera_info->K), 9, K.begin());
    cv::Mat_<value_t>	D(1, 4);
    std::copy_n(std::begin(_camera_info->D), 4, D.begin());
    cv::Mat_<point2_t>	uv(1, 1), xy(1, 1);
    uv(0) = {u, v};
    cv::undistortPoints(uv, xy, K, D);

    return {xy(0).x, xy(0).y, value_t(1)};
}


}	// namespace aist_localization
