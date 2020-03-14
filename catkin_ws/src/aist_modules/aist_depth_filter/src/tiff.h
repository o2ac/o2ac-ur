/*!
 *  \file	tiff.h
 *  \author	Toshio UESHIBA
 *  \brief	Save/Restore sensor_msgs::Image to/from TIFF file
 */
#ifndef TU_TIFF_H
#define TU_TIFF_H

#include <sensor_msgs/Image.h>
#include <string>

namespace aist_depth_filter
{
void			saveTiff(const sensor_msgs::Image& image,
				 const std::string& file)		;
sensor_msgs::ImagePtr	loadTiff(const std::string& file)		;
}	// namespace aist_depth_filter
#endif TU_TIFF_H
