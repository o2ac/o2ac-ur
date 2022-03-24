/*!
 *  \file	utils.h
 *  \author	Toshio UESHIBA
 *  \brief	Utilities
 */
#ifndef TU_UTILS_H
#define TU_UTILS_H

#include <array>
#include <algorithm>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace aist_depth_filter
{
//! Get intensity value
template <class T>
inline float	intensity(const T& grey)	{ return grey; }
template <>
inline float	intensity(const std::array<uint8_t, 3>& rgb)
		{
		    return 0.299f*rgb[0] + 0.587f*rgb[1] + 0.114f*rgb[2];
		}

//! Get depth value in meters.
template <class T>
inline float	meters(T depth)			{ return depth; }
template <>
inline float	meters(uint16_t depth)		{ return 0.001f * depth; }

//! Get depth value in milimeters.
template <class T>
inline float	milimeters(T depth)		{ return 1000.0f * depth; }
template <>
inline float	milimeters(uint16_t depth)	{ return depth; }

//! Get pointer to the leftmost pixel of v-th scanline of the image.
template <class T> inline T*
ptr(sensor_msgs::Image& image, int v)
{
    return reinterpret_cast<T*>(image.data.data() + v*image.step);
}

//! Get const pointer to the leftmost pixel of v-th scanline of the image.
template <class T> inline const T*
ptr(const sensor_msgs::Image& image, int v)
{
    return reinterpret_cast<const T*>(image.data.data() + v*image.step);
}

//! Convert depth image to sequence of 3D points.
template <class T, class ITER, class UNIT> ITER
depth_to_points(const sensor_msgs::CameraInfo& camera_info,
		const sensor_msgs::Image& depth, ITER out, UNIT unit)
{
    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info.K), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info.D), 4, D.begin());

    cv::Mat_<cv::Point2f>	uv(depth.width, 1), xy(depth.width, 1);
    for (int u = 0; u < depth.width; ++u)
	uv(u).x = u;

    for (int v = 0; v < depth.height; ++v)
    {
	for (int u = 0; u < depth.width; ++u)
	    uv(u).y = v;

	cv::undistortPoints(uv, xy, K, D);

	auto	p = ptr<T>(depth, v);
	for (int u = 0; u < depth.width; ++u)
	{
	    const auto	d = unit(*p++);		// meters or milimeters
	    *out++ = {xy(u).x * d, xy(u).y * d, d};
	}
    }

    return out;
}

//! Exterior product of two vectors.
template <class T, int M, int N> cv::Matx<T, M, N>
operator %(const cv::Matx<T, M, 1>& x, const cv::Matx<T, N, 1>& y)
{
    cv::Matx<T, M, N>	mat;
    for (size_t i = 0; i < M; ++i)
	for (size_t j = 0; j < N; ++j)
	    mat(i, j) = x(i) * y(j);
    return mat;
}

template <class IN> sensor_msgs::PointCloud2
create_pointcloud(IN in, IN ie,
		  const ros::Time& stamp, const std::string& frame)
{
    using namespace	sensor_msgs;

    PointCloud2	cloud;
    cloud.is_bigendian	  = false;
    cloud.is_dense	  = true;
    cloud.header.stamp	  = stamp;
    cloud.header.frame_id = frame;
    cloud.height	  = 1;
    cloud.width		  = std::distance(in, ie);

    PointCloud2Modifier	modifier(cloud);
    modifier.setPointCloud2Fields(3,
				  "x", 1, PointField::FLOAT32,
				  "y", 1, PointField::FLOAT32,
				  "z", 1, PointField::FLOAT32);
    modifier.resize(cloud.width);
    cloud.row_step = cloud.width * cloud.point_step;

    PointCloud2Iterator<float>	out(cloud, "x");
    for (; in != ie; ++in, ++out)
    {
	const auto&	xyz = *in;
	out[0] = xyz(0);
	out[1] = xyz(1);
	out[2] = xyz(2);
    }

    return cloud;
}

}	// namespace aist_depth_filter
#endif	// !TU_UTILS_H
