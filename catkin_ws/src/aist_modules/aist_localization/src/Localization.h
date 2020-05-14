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
#include <std_srvs/Trigger.h>
#include <aist_localization/LoadPlcf.h>
#include <aist_localization/LocalizeAction.h>
#include <aist_depth_filter/FileInfo.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

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

    using action_t	 = aist_localization::LocalizeAction;
    using feedback_t	 = aist_localization::LocalizeFeedback;
    using result_t	 = aist_localization::LocalizeResult;
    using goal_cp	 = aist_localization::LocalizeGoalConstPtr;
    using server_t	 = actionlib::SimpleActionServer<action_t>;

    using file_info_t	 = aist_depth_filter::FileInfo;
    using file_info_cp	 = aist_depth_filter::FileInfoConstPtr;

    struct setting_base
    {
	static std::string
			get_path(const std::string& name)
			{
			    auto	path = "User/" + name;
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

    template <class T>
    struct setting : public setting_base
    {
			setting(const std::string& name,
				const localization_t& localization)
			    :setting_base(name), value()
			{
			    get_from(localization);
			}

	virtual void	set_to(localization_t& localization) const
			{
			    localization.setSetting(path, value);

			    // ROS_INFO_STREAM('\"' << path << "\" <-- " << value);
			}

	virtual void	get_from(const localization_t& localization)
			{
			    value = localization.getSetting<T>(path);

			    // ROS_INFO_STREAM('\"' << path << "\" --> " << value);
			}

	T		value;
    };

  public:
		Localization(const std::string& name)			;

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
    bool	load_plcf_cb(LoadPlcf::Request&  req,
			     LoadPlcf::Response& res)			;
    bool	save_plcf_cb(std_srvs::Trigger::Request&  req,
			     std_srvs::Trigger::Response& res)		;
    void	file_info_cb(const file_info_cp& file_info)		;
    void	localize_cb(const goal_cp& goal)			;
    void	preempt_cb()					  const	;
    void	publish_feedback(const pho::sdk::LocalizationPose& locPose,
				 ros::Time time,
				 const std::string& object_frame) const	;

  private:
    ros::NodeHandle				_nh;

    std::string					_camera_frame;
    std::string					_plcf_dir;
    std::string					_object_name;

    std::unique_ptr<localization_t>		_localization;
    pho::sdk::SceneSource			_scene;
    bool					_is_valid_scene;

    ros::Subscriber				_file_info_sub;
    const ros::ServiceServer			_load_plcf_srv;
    const ros::ServiceServer			_save_plcf_srv;
    server_t					_localize_srv;

    ddynamic_reconfigure::DDynamicReconfigure	_ddr;
    std::vector<std::unique_ptr<setting_base> >	_settings;
};

}	// namespace aist_localization
#endif	// LOCALIZATION_H
