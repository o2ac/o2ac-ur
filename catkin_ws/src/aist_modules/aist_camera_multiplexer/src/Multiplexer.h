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
 *  \file	Multiplexer.h
 *  \author	Toshio Ueshiba
 *  \brief	Multiplexer for cameras
 */
#ifndef MULTIPLEXER_H
#define MULTIPLEXER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <aist_camera_multiplexer/ActivateCamera.h>

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
	const std::string		_camera_name;
	image_transport::ImageTransport	_it;
	image_transport::Subscriber	_image_sub;
	image_transport::Subscriber	_depth_sub;
	image_transport::Subscriber	_normal_sub;
	ros::Subscriber			_camera_info_sub;
    };

    using subscribers_cp = std::shared_ptr<const Subscribers>;

  public:
    Multiplexer(const ros::NodeHandle& nh)				;

    void	run()							;

  private:
    int		ncameras()					const	;
    void	activate_camera(int camera_number)			;
    bool	activate_camera_cb(ActivateCamera::Request&  req,
				   ActivateCamera::Response& res)	;
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
