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

    using action_t	 = aist_localization::LocalizeAction;
    using feedback_t	 = aist_localization::LocalizeFeedback;
    using result_t	 = aist_localization::LocalizeResult;
    using goal_cp	 = aist_localization::LocalizeGoalConstPtr;
    using server_t	 = actionlib::SimpleActionServer<action_t>;

    using file_info_t	 = aist_depth_filter::FileInfo;
    using file_info_cp	 = aist_depth_filter::FileInfoConstPtr;

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
    void	file_info_cb(const file_info_cp& file_info)		;
    void	localize_cb(const goal_cp& goal)			;
    void	preempt_cb()					  const	;

    void	localize_full(const goal_cp& goal)			;
    void	localize_bbox_center(const goal_cp& goal)		;
    void	localize_bbox_diagonal(const goal_cp& goal)		;
    vector3_t	view_vector(value_t u, value_t v)		  const	;

  private:
    ros::NodeHandle				_nh;

    std::string					_ply_dir;
    std::unique_ptr<localization_t>		_localization;

    ros::Subscriber				_file_info_sub;
    file_info_cp				_file_info;

    server_t					_localize_srv;

    ddynamic_reconfigure::DDynamicReconfigure	_ddr;
    std::vector<std::unique_ptr<setting_base> >	_settings;
};

}	// namespace aist_localization
#endif	// LOCALIZATION_H