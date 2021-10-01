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
 *  \file	DepthFilter.cpp
 *  \author	Toshio Ueshiba
 *  \brief	ROS node for applying filters to depth images
 */
#include <cstdlib>	// for getenv()
#include <sys/stat.h>	// for mkdir()
#include <sensor_msgs/image_encodings.h>
#include "tiff.h"
#include "ply.h"
#include "utils.h"
#include "binarize.h"
#include "ransac.h"
#include "DepthFilter.h"

namespace aist_depth_filter
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T, class ITER> ITER
get_dark_pixels(const sensor_msgs::Image& image, ITER iter)
{
  // Create an array of intensity values.
    std::vector<float>	intensities(image.height * image.width);
    auto		q = intensities.begin();
    for (size_t v = 0; v < image.height; ++v)
    {
	const auto	p = ptr<T>(image, v);
	q = std::transform(p, p + image.width, q, intensity<T>);
    }

  // Binarize intensities with Otsu's method
    const auto	thresh = *TU::binarize(intensities.begin(), intensities.end());

  // Extract pixels with itensities below the threshold
    auto	out = iter;
    for (size_t v = 0; v < image.height; ++v)
    {
	auto	p = ptr<T>(image, v);
	for (const auto pe = p + image.width; p != pe; ++p, ++iter)
	    if (intensity(*p) < thresh && (*iter)(2) != 0.0f)
		*out++ = *iter;
    }

    return out;		// past-the-end of the dark pixels
}

template <class T> inline bool
is_valid(T val)
{
    return (val != T(0) && !std::isnan(val));
}

/************************************************************************
*  class DepthFilter							*
************************************************************************/
DepthFilter::DepthFilter(const ros::NodeHandle& nh)
    :_nh(nh),
     _saveBG_srv(_nh.advertiseService("saveBG", &saveBG_cb, this)),
     _capture_srv(_nh.advertiseService("capture", &capture_cb, this)),
     _it(_nh),
     _camera_info_sub(_nh, "/camera_info", 1),
     _image_sub( _it, "/image",  1),
     _depth_sub( _it, "/depth",  1),
     _normal_sub(_it, "/normal", 1),
     _sync(sync_policy_t(10),
	   _camera_info_sub, _image_sub, _depth_sub, _normal_sub),
     _sync2(sync_policy2_t(10),
     	    _camera_info_sub, _image_sub, _depth_sub),
     _image_pub (_it.advertise("image",  1)),
     _depth_pub( _it.advertise("depth",  1)),
     _normal_pub(_it.advertise("normal", 1)),
     _colored_normal_pub(_it.advertise("colored_normal", 1)),
     _plane_pub(_nh.advertise<cloud_t>("base_plane", 1)),
     _camera_info_pub(_nh.advertise<camera_info_t>("camera_info", 1)),
     _file_info_pub(_nh.advertise<file_info_t>("file_info", 1)),
     _detect_plane_srv(_nh, "detect_plane",
		       boost::bind(&DepthFilter::detect_plane_cb, this, _1),
		       false),
     _ddr(_nh),
     _camera_info_org(nullptr),
     _camera_info(),
     _image_org(nullptr),
     _image(),
     _depth_org(nullptr),
     _depth_bg(nullptr),
     _depth(),
     _normal(),
     _threshBG(0.0),
     _near(0.0),
     _far(FarMax),
     _top(0),
     _bottom(2048),
     _left(0),
     _right(3072),
     _scale(1.0),
     _window_radius(0),
     _threshPlane(0.001)
{
  // Setup DetectPlane action server.
    _detect_plane_srv.registerPreemptCallback(boost::bind(&preempt_cb, this));
    _detect_plane_srv.start();

  // Setup parameters
    _nh.param("thresh_bg", _threshBG, _threshBG);
    _ddr.registerVariable<double>("thresh_bg", _threshBG,
				  boost::bind(
				      &DepthFilter::setVariable<double>,
				      this, &DepthFilter::_threshBG, _1),
				  "Threshold value for background removal",
				  0.0, 0.1);
    _nh.param("near", _near, _near);
    _ddr.registerVariable<double>("near", _near,
				  boost::bind(
				      &DepthFilter::setVariable<double>,
				      this, &DepthFilter::_near, _1),
				  "Nearest depth value", 0.0, 1.0);
    _nh.param("far", _far, _far);
    _ddr.registerVariable<double>("far", _far,
				  boost::bind(
				      &DepthFilter::setVariable<double>,
				      this, &DepthFilter::_far, _1),
				  "Farest depth value", 0.0, FarMax);
    _nh.param("top", _top, _top);
    _ddr.registerVariable<int>("top", _top,
			       boost::bind(&DepthFilter::setVariable<int>,
					   this, &DepthFilter::_top, _1),
			       "Top of ROI", 0, 2048);
    _nh.param("bottom", _bottom, _bottom);
    _ddr.registerVariable<int>("bottom", _bottom,
			       boost::bind(&DepthFilter::setVariable<int>,
					   this, &DepthFilter::_bottom, _1),
			       "Bottom of ROI", 0, 2048);
    _nh.param("left", _left, _left);
    _ddr.registerVariable<int>("left", _left,
			       boost::bind(&DepthFilter::setVariable<int>,
					   this, &DepthFilter::_left, _1),
			       "Left of ROI", 0, 3072);
    _nh.param("right", _right, _right);
    _ddr.registerVariable<int>("right", _right,
			       boost::bind(&DepthFilter::setVariable<int>,
					   this, &DepthFilter::_right, _1),
			       "Right of ROI", 0, 3072);
    _nh.param("scale", _scale, _scale);
    _ddr.registerVariable<double>("scale", _scale,
				  boost::bind(
				      &DepthFilter::setVariable<double>,
				      this, &DepthFilter::_scale, _1),
				  "Scale depth", 0.5, 1.5);
    _nh.param("thresh_plane", _threshPlane, _threshPlane);
    _ddr.registerVariable<double>("thresh_plane", &_threshPlane,
				  "Threshold of plane fitting", 0.0, 0.01);

    bool	subscribe_normal;
    _nh.param("subscribe_normal", subscribe_normal, true);
    if (subscribe_normal)
    {
	_sync.registerCallback(&DepthFilter::filter_with_normal_cb, this);
    }
    else
    {
	_nh.param("window_radius", _window_radius, 0);
	_ddr.registerVariable<int>("window_radius", _window_radius,
				   boost::bind(
				       &DepthFilter::setVariable<int>,
				       this, &DepthFilter::_window_radius, _1),
				   "Window radius", 0, 5);
	_sync2.registerCallback(&DepthFilter::filter_without_normal_cb, this);
    }

    _ddr.publishServicesTopics();
}

void
DepthFilter::run()
{
    ros::spin();
}

template <class T> void
DepthFilter::setVariable(T DepthFilter::* p, T value)
{
    this->*p = value;
    _depth.data.clear();	// Invalidate old depth data.
}

bool
DepthFilter::saveBG_cb(std_srvs::Trigger::Request&  req,
		       std_srvs::Trigger::Response& res)
{
    try
    {
	if (!_depth_org)
	    throw std::runtime_error("no original depth image available!");

	saveTiff(*_depth_org, open_dir() + "/bg.tif");

	_depth_bg  = _depth_org;
	_depth_org = nullptr;

	res.success = true;
	res.message = "succeeded.";
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::saveBG_cb(): " << err.what());

	res.success = false;
	res.message = "failed.";
    }

    ROS_INFO_STREAM("(DepthFilter) save background image: " << res.message);

    return true;
}

bool
DepthFilter::capture_cb(std_srvs::Trigger::Request&  req,
			std_srvs::Trigger::Response& res)
{
    try
    {
	if (_depth.data.empty())
	    throw std::runtime_error("no filtered depth image available!");

	const auto	file_path = open_dir() + "/scene.ply";
	savePly(_camera_info, _image, _depth, _normal, file_path);
	_depth.data.clear();

	file_info_t	file_info;
	file_info.file_path = file_path;
	file_info.header    = _camera_info.header;

	if (_threshPlane > 0.0)
	{
	    const auto	plane = detect_plane(*_camera_info_org,
					     *_image_org, *_depth_org,
					     _threshPlane);
	    file_info.plane_detected = true;
	    file_info.normal.x = plane.normal()(0);
	    file_info.normal.y = plane.normal()(1);
	    file_info.normal.z = plane.normal()(2);
	    file_info.distance = plane.distance();
	}
	else
	{
	    file_info.plane_detected = false;
	    file_info.normal.x = 0;
	    file_info.normal.y = 0;
	    file_info.normal.z = -1;
	    file_info.distance = 0;
	}

	_file_info_pub.publish(file_info);

	res.success = true;
	res.message = "succeeded.";

	ROS_INFO_STREAM("(DepthFilter) save as OrderedPly: " << res.message);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(DepthFilter) capture_cb(): " << err.what());

	res.success = false;
	res.message = "failed.";
    }

    return true;
}

void
DepthFilter::filter_with_normal_cb(const camera_info_cp& camera_info,
				   const image_cp& image,
				   const image_cp& depth,
				   const image_cp& normal)
{
    _top    = std::max(0,     std::min(_top,    int(image->height)));
    _bottom = std::max(_top,  std::min(_bottom, int(image->height)));
    _left   = std::max(0,     std::min(_left,   int(image->width)));
    _right  = std::max(_left, std::min(_right,  int(image->width)));

    if (_top == _bottom || _left == _right)
	return;

    try
    {
	using	namespace sensor_msgs;

      // Keep pointers to original data.
	_camera_info_org = camera_info;
	_image_org	 = image;
	_depth_org	 = depth;

      // Create camera_info according to ROI.
	_camera_info	    = *camera_info;
	_camera_info.height = _bottom - _top;
	_camera_info.width  = _right  - _left;
	_camera_info.K[2]  -= _left;
	_camera_info.K[5]  -= _top;
	_camera_info.P[2]  -= _left;
	_camera_info.P[6]  -= _top;

	create_subimage(*image,  _image);
	create_subimage(*depth,  _depth);
	create_subimage(*normal, _normal);
	create_colored_normal(_normal, _colored_normal);

	if (depth->encoding == image_encodings::MONO16 ||
	    depth->encoding == image_encodings::TYPE_16UC1)
	    filter<uint16_t>(_camera_info, _depth);
	else if (depth->encoding == image_encodings::TYPE_32FC1)
	    filter<float>(_camera_info, _depth);

	_camera_info_pub.publish(_camera_info);
	_image_pub.publish(_image);
	_depth_pub.publish(_depth);
	_normal_pub.publish(_normal);
	_colored_normal_pub.publish(_colored_normal);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::filter_with_normal_cb(): "
			 << err.what());
    }
}

void
DepthFilter::filter_without_normal_cb(const camera_info_cp& camera_info,
				      const image_cp& image,
				      const image_cp& depth)
{
    _top    = std::max(0,     std::min(_top,    int(image->height)));
    _bottom = std::max(_top,  std::min(_bottom, int(image->height)));
    _left   = std::max(0,     std::min(_left,   int(image->width)));
    _right  = std::max(_left, std::min(_right,  int(image->width)));

    if (_top == _bottom || _left == _right)
	return;

    try
    {
	using	namespace sensor_msgs;

      // Keep pointers to original data.
	_camera_info_org = camera_info;
	_image_org	 = image;
	_depth_org	 = depth;

      // Create camera_info according to ROI.
	_camera_info	    = *camera_info;
	_camera_info.height = _bottom - _top;
	_camera_info.width  = _right  - _left;
	_camera_info.K[2]  -= _left;
	_camera_info.K[5]  -= _top;
	_camera_info.P[2]  -= _left;
	_camera_info.P[6]  -= _top;

	create_subimage(*image, _image);
	create_subimage(*depth, _depth);

	if (depth->encoding == image_encodings::MONO16 ||
	    depth->encoding == image_encodings::TYPE_16UC1)
	    filter<uint16_t>(_camera_info, _depth);
	else if (depth->encoding == image_encodings::TYPE_32FC1)
	    filter<float>(_camera_info, _depth);

	_camera_info_pub.publish(_camera_info);
	_image_pub.publish(_image);
	_depth_pub.publish(_depth);
	_normal_pub.publish(_normal);
	_colored_normal_pub.publish(_colored_normal);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::filter_without_cb(): " << err.what());
    }
}

void
DepthFilter::preempt_cb() const
{
    _detect_plane_srv.setPreempted();
    ROS_INFO_STREAM("(DepthFilter)   *DetectPlaneAction preempted*");
}

void
DepthFilter::detect_plane_cb(const goal_cp& goal)
{
    try
    {
	if (!_camera_info_org || !_image_org || !_depth_org)
	    throw std::runtime_error("no images receved!");

	if (_threshPlane <= 0.0)
	    throw std::runtime_error("parameter threshPlane is not positive!");

	const auto	plane = detect_plane(*_camera_info_org,
					     *_image_org, *_depth_org,
					     _threshPlane);

	DetectPlaneResult	result;
	result.plane.header	    = _camera_info_org->header;
	result.plane.plane.normal.x = plane.normal()(0);
	result.plane.plane.normal.y = plane.normal()(1);
	result.plane.plane.normal.z = plane.normal()(2);
	result.plane.plane.distance = plane.distance();

	_detect_plane_srv.setSucceeded(result);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(DepthFilter) detect_plane_cb(): "
			 << err.what());
	_detect_plane_srv.setAborted();
    }
}

template <class T> void
DepthFilter::filter(const camera_info_t& camera_info, image_t& depth)
{
    if (_threshBG > 0)
    {
	try
	{
	    if (!_depth_bg)
		_depth_bg = loadTiff(open_dir() + "/bg.tif");

	    removeBG<T>(depth, *_depth_bg);
	}
	catch (const std::exception& err)
	{
	    _depth_bg = nullptr;
	    _threshBG = 0;
	}
    }
    if (_near > 0.0 || _far < FarMax)
    {
	z_clip<T>(depth);
    }
    if (_window_radius > 0)
    {
	computeNormal<T>(camera_info, depth);
    }
    if (_scale != 1.0)
    {
	scale<T>(depth);
    }
}

template <class T> void
DepthFilter::removeBG(image_t& depth, const image_t& depth_bg) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	auto	p = ptr<T>(depth, v);
	auto	b = ptr<T>(depth_bg, v + _top) + _left;
	for (const auto q = p + depth.width; p != q; ++p, ++b)
	    if (*b != 0 && std::abs(meters(*p) - meters(*b)) < _threshBG)
		*p = 0;
    }
}

template <class T> void
DepthFilter::z_clip(image_t& depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	const auto	p = ptr<T>(depth, v);
	std::replace_if(p, p + depth.width,
			[this](const auto& val)
			{ return (meters(val) < _near || meters(val) > _far); },
			0);
    }
}

template <class T> void
DepthFilter::computeNormal(const camera_info_t& camera_info,
			   const image_t& depth)
{
    using	namespace sensor_msgs;

  // Computation of normals should be done in double-precision
  // in order to avoid truncation error when sliding windows
    using normal_t		= std::array<value_t, 3>;
    using colored_normal_t	= std::array<uint8_t, 3>;
    using vector3_t		= cv::Vec<double, 3>;	   // double-precision
    using matrix33_t		= cv::Matx<double, 3, 3>;  // double-precision

  // 0: Allocate image for output normals.
    _normal.header		= depth.header;
    _normal.encoding		= image_encodings::TYPE_32FC3;
    _normal.height		= depth.height;
    _normal.width		= depth.width;
    _normal.step		= _normal.width * sizeof(normal_t);
    _normal.is_bigendian	= false;
    _normal.data.resize(_normal.height * _normal.step);
    std::fill(_normal.data.begin(), _normal.data.end(), 0);

  // 1: Allocate image for output colored normals.
    _colored_normal.header	 = depth.header;
    _colored_normal.encoding	 = image_encodings::RGB8;
    _colored_normal.height	 = depth.height;
    _colored_normal.width	 = depth.width;
    _colored_normal.step	 = _colored_normal.width
				 * sizeof(colored_normal_t);
    _colored_normal.is_bigendian = false;
    _colored_normal.data.resize(_colored_normal.height * _colored_normal.step);
    std::fill(_colored_normal.data.begin(), _colored_normal.data.end(), 0);

  // 2: Compute 3D coordinates.
    cv::Mat_<vector3_t>		xyz(depth.height, depth.width);
    depth_to_points<T>(camera_info, depth, xyz.begin(), milimeters<T>);

  // 3: Compute normals.
    const auto			ws1 = 2 * _window_radius;
    cv::Mat_<int>		n(depth.width - ws1, depth.height);
    cv::Mat_<vector3_t>		c(depth.width - ws1, depth.height);
    cv::Mat_<matrix33_t>	M(depth.width - ws1, depth.height);

  // 3.1: Convovle with a box filter in horizontal direction.
    for (int v = 0; v < n.cols; ++v)
    {
	auto		sum_n = 0;
	vector3_t	sum_c(0, 0, 0);
	auto		sum_M = matrix33_t::zeros();
	for (int u = 0; u < ws1; ++u)
	{
	    const auto&	head = xyz(v, u);

	    if (is_valid(head(2)))
	    {
		++sum_n;
		sum_c += head;
		sum_M += head % head;
	    }
	}

	for (int u = 0; u < n.rows; ++u)
	{
	    const auto&	head = xyz(v, u + ws1);

	    if (is_valid(head(2)))
	    {
		++sum_n;
		sum_c += head;
 		sum_M += head % head;
	    }

	    n(u, v) = sum_n;
	    c(u, v) = sum_c;
	    M(u, v) = sum_M;

	    const auto&	tail = xyz(v, u);

	    if (is_valid(tail(2)))
	    {
		--sum_n;
		sum_c -= tail;
		sum_M -= tail % tail;
	    }
	}
    }

  // 3.2: Convolve with a box filter in vertical direction.
    for (int u = 0; u < n.rows; ++u)
    {
	auto	sum_n = 0;
	auto	sum_c = vector3_t::zeros();
	auto	sum_M = matrix33_t::zeros();
	for (int v = 0; v < ws1; ++v)
	{
	    sum_n += n(u, v);
	    sum_c += c(u, v);
	    sum_M += M(u, v);
	}

	auto	norm  = ptr<normal_t>(_normal, _window_radius)
		      + _window_radius + u;
	auto	cnorm = ptr<colored_normal_t>(_colored_normal, _window_radius)
		      + _window_radius + u;
	for (int v = ws1; v < n.cols; ++v)
	{
	    sum_n += n(u, v);
	    sum_c += c(u, v);
	    sum_M += M(u, v);

	    if (sum_n > 3)
	    {
		const auto	A = sum_n * sum_M - sum_c % sum_c;
		vector3_t	evalues;
		matrix33_t	evectors;
		cv::eigen(A, evalues, evectors);	// Fit a plane.
		auto		normal   = evectors.row(2).t();
		auto		distance = normal.dot(sum_c) / sum_n;
		if (distance > 0)
		{
		    distance *= -1;
		    normal   *= -1;
		}

		// std::cerr << "evalues = ("
		// 	  << evalues(0) << ", "
		// 	  << evalues(1) << ", "
		// 	  << evalues(2) << ")" << std::endl;
		// std::cerr << "dist = " << dist << ", norm = ("
		// 	  << norm(0) << ", "
		// 	  << norm(1) << ", "
		// 	  << norm(2) << ")" << std::endl;

		*norm  = {normal(0), normal(1), normal(2)};
		*cnorm = {uint8_t(128 + 127*normal(0)),
			  uint8_t(128 + 127*normal(1)),
			  uint8_t(128 + 127*normal(2))};
	    }

	    const auto	vh = v - ws1;
	    sum_n -= n(u, vh);
	    sum_c -= c(u, vh);
	    sum_M -= M(u, vh);

	    norm  += _normal.width;
	    cnorm += _colored_normal.width;
	}
    }
}

template <class T> void
DepthFilter::scale(image_t& depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	const auto	p = ptr<T>(depth, v);
	std::transform(p, p + depth.width, p,
		       [this](const auto& val){ return _scale * val; });
    }
}

DepthFilter::plane_t
DepthFilter::detect_plane(const camera_info_t& camera_info,
			  const image_t& image,
			  const image_t& depth, float thresh) const
{
    using	namespace sensor_msgs;

    using grey_t	= uint8_t;
    using rgb_t		= std::array<uint8_t, 3>;
    using vector3_t	= cv::Vec<value_t, 3>;

  // Convert depths to 3D coordinates.
    std::vector<vector3_t>	xyz(depth.height * depth.width);
    if (depth.encoding == image_encodings::MONO16 ||
	depth.encoding == image_encodings::TYPE_16UC1)
	depth_to_points<uint16_t>(camera_info, depth, xyz.begin(),
				  meters<uint16_t>);
    else if (depth.encoding == image_encodings::TYPE_32FC1)
	depth_to_points<float>(camera_info, depth, xyz.begin(), meters<float>);
    else
	throw std::runtime_error("unknown dpeth encoding: " + depth.encoding);

  // Extract pixels with intesities below the threshold.
    const auto	end = (image.encoding == image_encodings::RGB8  ||
		       image.encoding == image_encodings::TYPE_8UC3 ?
    		       get_dark_pixels< rgb_t>(image, xyz.begin()) :
    		       get_dark_pixels<grey_t>(image, xyz.begin()));

  // Fit a dominant plane to the extract pixels.
    plane_t	plane;
    const auto	inliers = TU::ransac(xyz.begin(), end, plane,
				     [thresh](const auto& p, const auto& plane)
				     { return plane.distance(p) < thresh; },
				     value_t(0.5), value_t(0.99));

  // Create pointcloud and publish it.
    const auto	cloud = create_pointcloud(inliers.begin(), inliers.end(),
					  depth.header.stamp,
					  depth.header.frame_id);
    _plane_pub.publish(cloud);

    return plane;
}

void
DepthFilter::create_subimage(const image_t& image, image_t& subimage) const
{
    using	namespace sensor_msgs;

    const auto	nbytesPerPixel = image_encodings::bitDepth(image.encoding)/8
			       * image_encodings::numChannels(image.encoding);

    subimage.header	  = image.header;
    subimage.height	  = _bottom - _top;
    subimage.width	  = _right  - _left;
    subimage.encoding	  = image.encoding;
    subimage.is_bigendian = image.is_bigendian;
    subimage.step	  = subimage.width*nbytesPerPixel;
    subimage.data.resize(subimage.height * subimage.step);

    auto p = image.data.begin() + _top*image.step + _left*nbytesPerPixel;
    for (auto q = subimage.data.begin(); q != subimage.data.end();
	 q += subimage.step)
    {
	std::copy_n(p, subimage.width*nbytesPerPixel, q);
	p += image.step;
    }
}

void
DepthFilter::create_colored_normal(const image_t& normal,
				   image_t& colored_normal) const
{
    using	namespace sensor_msgs;

    using normal_t		= std::array<float, 3>;
    using colored_normal_t	= std::array<uint8_t, 3>;

    colored_normal.header	= normal.header;
    colored_normal.height	= normal.height;
    colored_normal.width	= normal.width;
    colored_normal.encoding	= image_encodings::RGB8;
    colored_normal.is_bigendian = normal.is_bigendian;
    colored_normal.step		= colored_normal.width
				* sizeof(colored_normal_t);
    colored_normal.data.resize(colored_normal.height * colored_normal.step);

    for (int v = 0; v < normal.height; ++v)
    {
	const auto	p = ptr<normal_t>(normal, v);
	const auto	q = ptr<colored_normal_t>(colored_normal, v);

	std::transform(p, p + normal.width, q,
		       [](const auto& norm)
		       {
			   return colored_normal_t(
				       {uint8_t(128 + 127*norm[0]),
					uint8_t(128 + 127*norm[1]),
				        uint8_t(128 + 127*norm[2])});
		       });
    }
}

std::string
DepthFilter::open_dir()
{
    const auto	home = getenv("HOME");
    if (!home)
	throw std::runtime_error("Environment variable[HOME] is not set.");

    const auto	dir_name = home + std::string("/.ros")
				+ ros::this_node::getNamespace();
    struct stat	buf;
    if (stat(dir_name.c_str(), &buf) && mkdir(dir_name.c_str(), S_IRWXU))
	throw std::runtime_error("Cannot create " + dir_name + ": "
						  + strerror(errno));

    return dir_name;
}

}	// namespace aist_depth_filter
