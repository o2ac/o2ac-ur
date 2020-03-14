/*!
 *  \file	utils.h
 *  \author	Toshio UESHIBA
 *  \brief	Utilities
 */
#ifndef TU_UTILS_H
#define TU_UTILS_H

#include <algorithm>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc.hpp>

namespace aist_depth_filter
{
//! Get depth value in meters.
template <class T>
inline float	fval(const T depth)		{ return depth; }
inline float	fval(const uint16_t depth)	{ return 0.001f * depth; }

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
template <class T, class ITER> ITER
depth_to_points(const sensor_msgs::CameraInfo& camera_info,
		const sensor_msgs::Image& depth, ITER out)
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
	    const auto	d = 1000.0f * fval(*p++);	// unit: milimeters
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

}	// namespace aist_depth_filter
#endif	// !TU_UTILS_H
