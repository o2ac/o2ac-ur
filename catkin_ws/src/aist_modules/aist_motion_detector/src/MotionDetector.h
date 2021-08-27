/*
 *  \file	MotionDetector.h
 *  \author	Toshio UESHIBA
 *  \brief	ROS node for tracking corners in 2D images
 */
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/bgsegm.hpp>
#include <aist_motion_detector/FindCabletipAction.h>
#include "Plane.h"

namespace aist_motion_detector
{
/************************************************************************
*  class MotionDetector							*
************************************************************************/
class MotionDetector
{
  private:
    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using image_t	 = sensor_msgs::Image;
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using image_p	 = sensor_msgs::ImagePtr;
    using sync_policy_t	 = message_filters::sync_policies::
				ApproximateTime<camera_info_t,
						image_t, image_t>;

    using bgsub_p	 = cv::Ptr<cv::BackgroundSubtractor>;

    using value_t	 = float;
    using point3_t	 = cv::Point3_<value_t>;
    using point2_t	 = cv::Point_<value_t>;
    using point_t	 = cv::Point;
    using vector3_t	 = cv::Vec<value_t, 3>;
    using line_t	 = TU::Plane<value_t, 2>;
    using plane_t	 = TU::Plane<value_t, 3>;
    
  public:
		MotionDetector(const ros::NodeHandle& nh)		;
		~MotionDetector()					;

    void	run()							;

  private:
  // action callbacks
    void	goal_cb()						;
    void	preempt_cb()						;

  // topic callbacks
    void	image_cb(const camera_info_cp& camera_info,
			 const image_cp& image, const image_cp& depth)	;

  // ddynamic_reconfigure callbacks
    void	set_sequential_mode_cb(bool enable)			;
    template <class T>
    void	set_param_cb(T MotionDetector::* field, T value,
			     bool select)				;

  // utility functions
    void	accumulate_mask(const cv::Mat& mask,
				const std::string& target_frame,
				const camera_info_cp& camera_info)	;
    tf::Transform
		find_cabletip(const point_t& top_left)			;
    vector3_t	view_vector(value_t u, value_t v)		const	;
    
  private:
    ros::NodeHandle					_nh;

    image_transport::ImageTransport			_it;
    image_transport::SubscriberFilter			_image_sub;
    image_transport::SubscriberFilter			_depth_sub;
    message_filters::Subscriber<camera_info_t>		_camera_info_sub;
    message_filters::Synchronizer<sync_policy_t>	_sync;
    const image_transport::CameraPublisher		_camera_pub;
    const image_transport::Publisher			_image_pub;
    const image_transport::Publisher			_mask_pub;

    const tf::TransformListener				_listener;
    tf::TransformBroadcaster				_broadcaster;
    
    actionlib::SimpleActionServer<FindCabletipAction>	_find_cabletip_srv;
    FindCabletipGoalConstPtr				_current_goal;

  // Motion detector parameters and dynamic_reconfigure server for setting them
    ddynamic_reconfigure::DDynamicReconfigure		_ddr;

  // Motion detector stuffs
    bgsub_p						_bgsub;
    double						_search_top;
    double						_search_bottom;
    double						_search_width;

    int							_nframes;
    point_t						_top_left;
    cv::Mat_<point2_t>					_corners;
    cv_bridge::CvImagePtr				_cv_image;
    cv_bridge::CvImage					_cv_accum;
    tf::StampedTransform				_Tct;
    camera_info_cp					_camera_info;
};

}	// namespace aist_motion_detector
