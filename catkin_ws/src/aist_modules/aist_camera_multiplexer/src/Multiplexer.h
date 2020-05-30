/*!
 *  \file	Multiplexer.h
 *  \author	Toshio UESHIBA
 *  \brief	Multiplexer for cameras
 */
#ifndef MULTIPLEXER_H
#define MULTIPLEXER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace aist_camera_multiplexer
{
/************************************************************************
*  class Multiplexer							*
************************************************************************/
class Multiplexer
{
  private:
    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using image_t	 = sensor_msgs::Image;
    using image_cp	 = sensor_msgs::ImageConstPtr;

    class SyncedSubscribers
    {
      private:
	using sync_policy_t = message_filters::sync_policies::
				ApproximateTime<camera_info_t,
						image_t, image_t, image_t>;

      public:
	SyncedSubscribers(Multiplexer* multiplexer, int camera_number)	;

      private:
	message_filters::Subscriber<camera_info_t>	_camera_info_sub;
	message_filters::Subscriber<image_t>		_image_sub;
	message_filters::Subscriber<image_t>		_depth_sub;
	message_filters::Subscriber<image_t>		_normal_sub;
	message_filters::Synchronizer<sync_policy_t>	_sync;
    };

    using subscribers_cp = std::shared_ptr<const SyncedSubscribers>;

  public:
    Multiplexer(const ros::NodeHandle& nh)				;

    void	run()							;

  private:
    void	activate_camera(int camera_number)			;
    void	synced_images_cb(const camera_info_cp& camera_info,
				 const image_cp& image,
				 const image_cp& depth,
				 const image_cp& normal,
				 int camera_number)		const	;

  private:
    ros::NodeHandle				_nh;

    std::vector<subscribers_cp>			_subscribers;
    int						_camera_number;

    ddynamic_reconfigure::DDynamicReconfigure	_ddr;

    image_transport::ImageTransport		_it;
    const image_transport::Publisher		_image_pub;
    const image_transport::Publisher		_depth_pub;
    const image_transport::Publisher		_normal_pub;
    const ros::Publisher			_camera_info_pub;
};

}	// namespace aist_camera_multiplexer
#endif	// MULTIPLEXER_H
