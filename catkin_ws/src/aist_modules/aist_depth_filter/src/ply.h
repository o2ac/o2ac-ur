/*!
 *  \file	ply.h
 *  \author	Toshio UESHIBA
 *  \brief	Save depth and color images to Ordered PLY file
 */
#ifndef TU_PLY_H
#define TU_PLY_H

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <string>

namespace aist_depth_filter
{
void	savePly(const sensor_msgs::CameraInfo& camera_info,
		const sensor_msgs::Image& image,
		const sensor_msgs::Image& depth,
		const sensor_msgs::Image& normal,
		const std::string& file)				;
}	// namespace aist_depth_filter
#endif TU_PLY_H
