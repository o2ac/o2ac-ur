/*!
 *  \file	Multiplexer.h
 *  \author	Toshio UESHIBA
 *  \brief	Multiplexer for cameras
 */
#ifndef MULTIPLEXER_H
#define MULTIPLEXER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
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

    class Subscribers
    {
      public:
	Subscribers(Multiplexer* multiplexer,
		    const std::string& camera_name			);

	const std::string&	camera_name()			const	;

      private:
	const std::string	_camera_name;
	ros::Subscriber		_camera_info_sub;
	ros::Subscriber		_image_sub;
	ros::Subscriber		_depth_sub;
	ros::Subscriber		_normal_sub;
    };

    using subscribers_cp = std::shared_ptr<const Subscribers>;

  public:
    Multiplexer(const ros::NodeHandle& nh)				;

    void	run()							;

  private:
    int		ncameras()					const	;
    void	activate_camera(int camera_number)			;
    void	camera_info_cb(const camera_info_cp& camera_info,
			       int camera_number)		const	;
    void	image_cb(const image_cp& image,
			 int camera_number)			const	;
    void	depth_cb(const image_cp& depth,
			 int camera_number)			const	;
    void	normal_cb(const image_cp& normal,
			  int camera_number)			const	;

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
