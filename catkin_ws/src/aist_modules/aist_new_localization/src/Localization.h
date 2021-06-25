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
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <aist_depth_filter/PlaneStamped.h>
#include <aist_new_localization/LocalizeAction.h>
#include <std_srvs/Trigger.h>
#include <opencv2/core.hpp>

namespace aist_new_localization
{
/************************************************************************
*  class Localization							*
************************************************************************/
class Localization
{
  private:
    using action_t	 = LocalizeAction;
    using feedback_t	 = LocalizeFeedback;
    using result_t	 = LocalizeResult;
    using goal_cp	 = LocalizeGoalConstPtr;
    using server_t	 = actionlib::SimpleActionServer<action_t>;

    using value_t	 = float;
    using vector3_t	 = cv::Vec<value_t, 3>;
    using plane_t	 = aist_depth_filter::PlaneStamped;

    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using image_t	 = sensor_msgs::Image;
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using image_p	 = sensor_msgs::ImagePtr;
    using cloud_t	 = sensor_msgs::PointCloud2;
    using sync_policy_t	 = message_filters::sync_policies::
				ApproximateTime<camera_info_t, image_t>;

  public:
		Localization(const ros::NodeHandle& nh)			;

    void	run()							;

  private:
    void	goal_cb()						;
    void	preempt_cb()						;
    void	localize_cb(const camera_info_cp& camera_info,
			    const image_cp& depth)			;

    void	transform_plane(const std::string& target_frame,
				const plane_t& plane,
				vector3_t& normal,
				value_t& distance)		const	;
    static vector3_t
		view_vector(const camera_info_cp& camera_info,
			    value_t u, value_t v)			;
    tf::Transform
		refine_transform(const std::string& object_name,
				 const tf::Transform& transform,
				 const camera_info_cp& camera_info,
				 const image_cp& depth,
				 value_t& error)		const	;

  private:
    ros::NodeHandle					_nh;

    const tf::TransformListener				_listener;

    message_filters::Subscriber<camera_info_t>		_camera_info_sub;
    message_filters::Subscriber<image_t>		_depth_sub;
    message_filters::Synchronizer<sync_policy_t>	_sync;

    const ros::Publisher				_model_cloud_pub;
    const ros::Publisher				_data_cloud_pub;

    server_t						_localize_srv;
    goal_cp						_current_goal;

    ddynamic_reconfigure::DDynamicReconfigure		_ddr;
    const std::string					_pcd_dir;
    int							_niterations;
    double						_max_distance;
    const double					_transformation_epsilon;
    const double					_fitness_epsilon;
};

}	// namespace aist_localization
#endif	// LOCALIZATION_H
