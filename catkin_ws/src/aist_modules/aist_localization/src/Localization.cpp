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
     _object_name(""),
     _localization(new localization_t()),
     _scene(),
     _is_valid_scene(false),
     _file_info_sub(_nh.subscribe<file_info_t>(
			"/file_info", 1, &Localization::file_info_cb, this)),
     _load_plcf_srv(_nh.advertiseService("load_plcf", &load_plcf_cb, this)),
     _save_plcf_srv(_nh.advertiseService("save_plcf", &save_plcf_cb, this)),
     _localize_srv(_nh, "localize",
		   boost::bind(&Localization::localize_cb, this, _1), false),
     _ddr(_nh)
{
    _localize_srv.registerPreemptCallback(
	boost::bind(&Localization::preempt_cb, this));
    _localize_srv.start();

    _nh.param("plcf_dir", _plcf_dir,
	      ros::package::getPath("aist_localization") + "/plcf");

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

    const auto	p = new setting<std::string>(name, *_localization);
    _settings.emplace_back(p);
    _ddr.registerEnumVariable<std::string>(name, &p->value,
					   description, enum_dict);
}

bool
Localization::load_plcf_cb(LoadPlcf::Request& req, LoadPlcf::Response& res)
{
    try
    {
	ROS_INFO_STREAM("(Localization) Loading configuration of model["
			<< req.object_name << "]...");

      // Load configuration.
	const auto plcf_file = _plcf_dir + '/' + req.object_name + ".plcf";
	if (!_localization->LoadLocalizationConfiguration(plcf_file))
	    throw std::runtime_error("  Failed to load configuration["
				     + plcf_file + "].");

      // Get settings from localizer and publish them.
	for (auto& setting : _settings)
	    setting->get_from(*_localization);
	_ddr.updatePublishedInformation();

	_object_name = req.object_name;
	res.success = true;

	ROS_INFO_STREAM("(Localization)   succeeded.");
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(Localization) " << err.what());

	_object_name = "";
	res.success = false;
    }

    return true;
}

bool
Localization::save_plcf_cb(std_srvs::Trigger::Request&  req,
			   std_srvs::Trigger::Response& res)
{
    try
    {
	if (_object_name == "")
	    throw std::runtime_error("Cannot save configuration because it is empty.");

	ROS_INFO_STREAM("(Localization) Saving configuration for object["
			<< _object_name << "]...");

	const auto	plcf_file = _plcf_dir + _object_name + ".plcf";
	if (!_localization->SaveLocalizationConfiguration(plcf_file))
	    throw std::runtime_error("  Failed to save configuration["
				     + plcf_file + "].");

	ROS_INFO_STREAM("(Localization)   completed.");
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(Localization) " << err.what());
    }
}

void
Localization::file_info_cb(const file_info_cp& file_info)
{
    try
    {
	ROS_INFO_STREAM("(Localization) Set frame[" << file_info->frame
			<< "] and loading scene["
			<< file_info->file_path << "]...");

	_camera_frame = file_info->frame;
	_scene = pho::sdk::SceneSource::File(file_info->file_path);
	_localization->SetSceneSource(_scene);
	_is_valid_scene = true;

	ROS_INFO_STREAM("(Localization)   succeeded.");
    }
    catch (const std::exception& err)
    {
	_is_valid_scene = false;

	ROS_ERROR_STREAM("(Localization)   failed: " << err.what());
    }
}

void
Localization::localize_cb(const goal_cp& goal)
{
    try
    {
	ROS_INFO_STREAM("(Localization) Localizing object...");

	if (!_is_valid_scene)
	    throw std::runtime_error("  Scene is not set.");
	if (_object_name == "")
	    throw std::runtime_error("  Object model is not set.");
	ROS_INFO_STREAM("(Localization)   object_name = " << _object_name);

      // Set settings to localizer.
	for (const auto& setting : _settings)
	    setting->set_to(*_localization);

	ROS_INFO_STREAM("(Localization)   setting completed succesfully.");

      // Set stop criteria.
	_localization->AddStopCriterion(
	    pho::sdk::StopCriterion::NumberOfResults(goal->number_of_poses));

	ROS_INFO_STREAM("(Localization)   stop citeria succesfully set.");

      // Execute localization.
	const auto	now = ros::Time::now();
	for (auto queue = _localization->StartAsync();
	     queue.Size() != goal->number_of_poses; )
	{
	    ROS_INFO_STREAM("(Localization)     "
			    << queue.Size() << "-th trial.");

	    if (_localize_srv.isPreemptRequested())
	    {
		_localization->StopAsync();
		ROS_INFO_STREAM("(Localization)   preempted.");
		return;
	    }

	    pho::sdk::LocalizationPose	locPose;
	    if (queue.GetNext(locPose, 500))	// Timeout = 500ms
	    {
		const auto object_frame = _object_name + '_'
					+ std::to_string(queue.Size() - 1);
		publish_feedback(locPose, now, object_frame);

		ROS_INFO_STREAM("(Localization)   found "
				<< queue.Size() << "-th pose.");
	    }
	}

	_localization->StopAsync();
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
Localization::preempt_cb() const
{
    _localize_srv.setPreempted();
}

void
Localization::publish_feedback(const pho::sdk::LocalizationPose& locPose,
			       ros::Time time,
			       const std::string& object_frame) const
{
    const auto	k   = 0.001;
    const auto&	mat = locPose.Transformation;
    const tf::Transform
		transform(tf::Matrix3x3(mat[0][0], mat[0][1], mat[0][2],
					mat[1][0], mat[1][1], mat[1][2],
					mat[2][0], mat[2][1], mat[2][2]),
			  tf::Vector3(k*mat[0][3], k*mat[1][3], k*mat[2][3]));
    feedback_t	feedback;
    feedback.pose.header.stamp	  = time;
    feedback.pose.header.frame_id = _camera_frame;
    tf::poseTFToMsg(transform, feedback.pose.pose);
    feedback.overlap = locPose.VisibleOverlap;
    _localize_srv.publishFeedback(feedback);
}

}	// namespace aist_localization
