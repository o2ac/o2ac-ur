/*!
 *  \file	Localization.h
 *  \author	Toshio UESHIBA
 *  \brief	ROS wrapper of Photoneo Localization SDK
 */
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <iostream>
#include <cstdint>
#include <memory>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <aist_localization/LocalizeAction.h>
#include <aist_depth_filter/FileInfo.h>

#include <PhoLocalization.h>

namespace aist_localization
{
/************************************************************************
*  class Localization							*
************************************************************************/
class Localization
{
  private:
    using action_t	= aist_localization::LocalizeAction;
    using feedback_t	= aist_localization::LocalizeFeedback;
    using result_t	= aist_localization::LocalizeResult;
    using goal_cp	= aist_localization::LocalizeGoalConstPtr;
    using server_t	= actionlib::SimpleActionServer<action_t>;

    using file_info_t	 = aist_depth_filter::FileInfo;
    using file_info_cp	 = aist_depth_filter::FileInfoConstPtr;

  public:
		Localization(const std::string& name)			;

    void	run()						  const	;

  private:
    void	file_info_cb(const file_info_cp& file_info)		;
    void	localize_cb(const goal_cp& goal)		  const	;
    void	preempt_cb()					  const	;
    void	publish_feedback(const pho::sdk::LocalizationPose& locPose,
				 ros::Time time,
				 const std::string& object_frame) const	;

  private:
    ros::NodeHandle				_nh;

    std::string					_camera_frame;
    std::string					_plcf_dir;

    std::unique_ptr<pho::sdk::PhoLocalization>	_localization;
    pho::sdk::SceneSource			_scene;
    bool					_scene_is_valid;

    ros::Subscriber				_file_info_sub;
    server_t					_localize_srv;
};

}	// namespace aist_localization
#endif	// LOCALIZATION_H
