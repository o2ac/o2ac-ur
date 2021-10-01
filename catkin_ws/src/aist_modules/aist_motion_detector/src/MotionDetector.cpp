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
 *  \file	MotionDetector.cpp
 *  \author	Toshio Ueshiba
 *  \brief	ROS node for tracking corners in 2D images
 */
#include "MotionDetector.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/bgsegm.hpp>

namespace aist_motion_detector
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> static inline cv::Point3_<T>
pointTFToCV(const tf::Point& p)
{
    return {p.x(), p.y(), p.z()};
}

template <class T> static inline cv::Vec<T, 3>
vector3TFToCV(const tf::Vector3& v)
{
    return {v.x(), v.y(), v.z()};
}

template <class T> static inline tf::Vector3
vector3CVToTF(const cv::Vec<T, 3>& v)
{
    return {v(0), v(1), v(2)};
}

static inline bool
withinImage(const cv::Point& p, const cv::Mat& image)
{
    return (0 <= p.x && p.x < image.cols && 0 <= p.y && p.y < image.rows);
}

template <class T> T
distance(const cv::Mat& image, int label, const TU::Plane<T, 2>& line)
{
    auto	dmin = std::numeric_limits<T>::max();

    for (int v = 0; v < image.rows; ++v)
    {
	auto	p = image.ptr<int>(v, 0);

	for (int u = 0; u < image.cols; ++u, ++p)
	    if (*p == label)
	    {
		const auto	d = line.distance({u, v});
		if (d < dmin)
		    dmin = d;
	    }
    }

    return dmin;
}

static int
findLargestRegion(const cv::Mat& labels, const cv::Mat& stats, int nlabels,
		  const TU::Plane<float, 2>& finger_tip)
{
    int		amax = 0;
    auto	lmax = 0;
    for (int label = 1; label < nlabels; ++label)
    {
	const auto	d = distance(labels, label, finger_tip);

	if (d < 2)
	{
	    using cc_t = cv::ConnectedComponentsTypes;

	    const auto	stat = stats.ptr<int>(label);
	    const auto	area = stat[cc_t::CC_STAT_AREA];

	    if (area > amax)
	    {
		amax = area;
		lmax = label;
	    }
	}
    }

    return lmax;
}

template <class T> static cv::Vec<T, 2>
crossPoint(const TU::Plane<T, 2>& l1, const TU::Plane<T, 2>& l2)
{
    const cv::Vec<T, 3>	h1(l1.normal()(0), l1.normal()(1), l1.distance());
    const cv::Vec<T, 3>	h2(l2.normal()(0), l2.normal()(1), l2.distance());
    const auto		p = h1.cross(h2);

    return {p[0]/p[2], p[1]/p[2]};
}

static std::ostream&
operator <<(std::ostream& out, const tf::Vector3& v)
{
    return out << ' ' << v.x() << ' ' << v.y() << ' ' << v.z() << std::endl;
}

/************************************************************************
*  class MotionDetector							*
************************************************************************/
MotionDetector::MotionDetector(const ros::NodeHandle& nh)
    :_nh(nh),
     _it(_nh),
     _camera_info_sub(_nh, "/camera_info", 1),
     _image_sub(_it, "/image", 1),
     _depth_sub(_it, "/depth", 1),
     _sync(sync_policy_t(10), _camera_info_sub, _image_sub, _depth_sub),
     _image_pub(_it.advertise("image", 1)),
     _camera_pub(_it.advertiseCamera("depth", 1)),
     _listener(),
     _broadcaster(),
     _find_cabletip_srv(_nh, "find_cabletip", false),
     _current_goal(nullptr),
     _ddr(_nh),
     _bgsub(cv::createBackgroundSubtractorMOG2()),
     _search_top(_nh.param("search_top", 0.005)),
     _search_bottom(_nh.param("search_bottom", 0.030)),
     _search_width(_nh.param("search_width", 0.050))
{
  // Setup FindCabletip action server.
    _find_cabletip_srv.registerGoalCallback(boost::bind(&goal_cb, this));
    _find_cabletip_srv.registerPreemptCallback(boost::bind(&preempt_cb, this));
    _find_cabletip_srv.start();

  // Setup callback for synced camera_info and depth.
    _sync.registerCallback(&image_cb, this);

  // Setup parameters and ddynamic_reconfigure server.
    _ddr.registerVariable<double>("search_top", &_search_top,
				  "Top of search area in meters",
				  -0.010, 0.010);
    _ddr.registerVariable<double>("search_bottom", &_search_bottom,
				  "Bottom of search area in meters",
				  0.005, 0.050);
    _ddr.registerVariable<double>("search_width", &_search_width,
				  "Width of search area in meters",
				  0.005, 0.080);
    _ddr.publishServicesTopics();
}

MotionDetector::~MotionDetector()
{
}

void
MotionDetector::run()
{
    ros::spin();
}

void
MotionDetector::goal_cb()
{
    _current_goal = _find_cabletip_srv.acceptNewGoal();
    ROS_INFO_STREAM("(MotionDetector) Given a goal["
		    << _current_goal->target_frame << ']');
}

void
MotionDetector::preempt_cb()
{
    _find_cabletip_srv.setPreempted();
    ROS_INFO_STREAM("(MotionDetector) Cancelled a goal");
}

void
MotionDetector::image_cb(const camera_info_cp& camera_info,
			 const image_cp& image, const image_cp& depth)
{
    const auto	cv_image = cv_bridge::toCvCopy(image);

    if (_find_cabletip_srv.isActive())
    {
	try
	{
	    const auto		cv_depth = cv_bridge::toCvCopy(depth);
	    const auto		T = find_cabletip(cv_image->image,
						  cv_depth->image,
						  _current_goal->target_frame,
						  camera_info);
	    geometry_msgs::Pose	pose;
	    tf::poseTFToMsg(T, pose);

	    FindCabletipFeedback	feedback;
	    feedback.pose.header.frame_id = _current_goal->target_frame;
	    feedback.pose.header.stamp    = ros::Time::now();
	    feedback.pose.pose		  = pose;
	    _find_cabletip_srv.publishFeedback(feedback);

	    _broadcaster.sendTransform({T, feedback.pose.header.stamp,
					feedback.pose.header.frame_id,
					"cabletip_link"});

	    _camera_pub.publish(cv_depth->toImageMsg(), camera_info);
	}
	catch (const tf::TransformException& err)
	{
	    ROS_ERROR_STREAM("(MotionDetector) TransformException: "
			     << err.what());
	}
	catch (const cv_bridge::Exception& err)
	{
	    ROS_ERROR_STREAM("(MotionDetector) cv_bridge exception: "
			     << err.what());
	}
	catch (const std::exception& err)
	{
	    ROS_ERROR_STREAM("(MotionDetector) " << err.what());
	}
    }

    _image_pub.publish(cv_image->toImageMsg());
}

tf::Transform
MotionDetector::find_cabletip(cv::Mat& image, cv::Mat& depth,
			      const std::string& target_frame,
			      const camera_info_cp& camera_info) const
{
  // Create foregroud mask.
    cv::Mat	mask;
    _bgsub->apply(image, mask);

  // Binarize mask image.
    for (int v = 0; v < mask.rows; ++v)
    {
	auto	p = mask.ptr<uint8_t>(v, 0);

	for (const auto pe = p + mask.cols; p < pe; ++p)
	    if (*p < 255)
		*p = 0;
    }

  // Get transform from the target frame to camera frame.
    tf::StampedTransform	Tct;
    _listener.waitForTransform(camera_info->header.frame_id, target_frame,
			       camera_info->header.stamp, ros::Duration(1.0));
    _listener.lookupTransform(camera_info->header.frame_id, target_frame,
			      camera_info->header.stamp, Tct);

  // Transfrom ROI corners to camera frame.
    const tf::Point	pts[] = {{_search_top,    0.0, -_search_width/2},
				 {_search_top,    0.0,  _search_width/2},
				 {_search_bottom, 0.0,  _search_width/2},
				 {_search_bottom, 0.0, -_search_width/2}};
    cv::Mat_<point3_t>	pt3s(1, 4);
    for (size_t i = 0; i < 4; ++i)
	pt3s(i) = pointTFToCV<value_t>(Tct * pts[i]);

  // Project ROI corners onto the image.
    cv::Mat_<value_t>	K(3, 3);
    std::copy_n(std::begin(camera_info->K), 9, K.begin());
    cv::Mat_<value_t>	D(1, 4);
    std::copy_n(std::begin(camera_info->D), 4, D.begin());
    const auto		zero = cv::Mat_<value_t>::zeros(1, 3);
    cv::Mat_<point2_t>	pt2s(1, 4);
    cv::projectPoints(pt3s, zero, zero, K, D, pt2s);
    point_t		corners[] = {point_t(pt2s(0)), point_t(pt2s(1)),
				     point_t(pt2s(2)), point_t(pt2s(3))};

  // Confirm that all the projected corners are included within the image.
    if (!withinImage(corners[0], mask) || !withinImage(corners[1], mask) ||
	!withinImage(corners[2], mask) || !withinImage(corners[3], mask))
	throw std::runtime_error("ROI in 3D space projects onto the outside of image");

  // Fill the outside of ROI with zero.
    const point_t	outer[]	  = {{0,	   0},
				     {mask.cols-1, 0},
				     {mask.cols-1, mask.rows-1},
				     {0,	   mask.rows-1}};
    const point_t*	borders[] = {outer, corners};
    const int		npoints[] = {4, 4};
    cv::fillPoly(mask, borders, npoints, 2, cv::Scalar(0), cv::LINE_8);

  // Paint motion mask over the input RGB image.
    for (int v = 0; v < mask.rows; ++v)
    {
	auto	p = mask.ptr<uint8_t>(v, 0);
	auto	q = image.ptr<cv::Vec3b>(v, 0);

	for (const auto pe = p + mask.cols; p < pe; ++p, ++q)
	    if (*p >= 255)
		(*q)[1] = 255;
    }

  // Create a line of finger-tip border.
    const cv::Vec<value_t, 2>	ends[] = {pt2s(0), pt2s(1)};
    const line_t		fingertip(ends, ends + 2);

  // Assign labels to the binarized mask image.
    cv::Mat	labels, stats, centroids;
    const auto	nlabels = cv::connectedComponentsWithStats(mask, labels,
							   stats, centroids);

  // Find a largest region close to the finger-tip border.
    const auto	lmax = findLargestRegion(labels, stats, nlabels, fingertip);

  // Fit a line to the points in the region.
    std::vector<cv::Vec<value_t, 2> >	cable_points;
    for (int v = 0; v < labels.rows; ++v)
    {
	auto	p = labels.ptr<int>(v, 0);
	auto	q = mask.ptr<uint8_t>(v, 0);

	for (int u = 0; u < labels.cols; ++u, ++p, ++q)
	    if (*p == lmax)
	    {
		cable_points.push_back({u, v});
		*q = 255;
	    }
	    else
		*q = 0;
    }
    line_t	cable(cable_points.begin(), cable_points.end());

  // Compute cross point between finger-tip border and cable.
    const auto	root = crossPoint(fingertip, cable);

  // Find a point in the region farest from the cross point.
    value_t	dmax = 0;
    point2_t	cabletip;
    for (const auto& cable_point : cable_points)
    {
	const auto	p = cable.projection(cable_point);
	const auto	d = cv::norm(p, root);
	if (d > dmax)
	{
	    dmax     = d;
	    cabletip = p;
	}
    }

  // Plane including fingertip described w.r.t. camera frame.
    const auto		nc = vector3TFToCV<value_t>(Tct.getBasis()
						    .getColumn(1));
    const auto		tc = vector3TFToCV<value_t>(Tct.getOrigin());
    const plane_t	plane(nc, -nc.dot(tc));

  // Compute cross point between the plane and view vectors
  // for root and fingertip.
    const auto		root3 = Tct.inverse()
			      * vector3CVToTF(plane.cross_point(
						  view_vector(camera_info,
							      root(0),
							      root(1))));
    const auto		tip3  = Tct.inverse()
			      * vector3CVToTF(plane.cross_point(
						  view_vector(camera_info,
							      cabletip.x,
							      cabletip.y)));
    const auto		rx = (tip3 - root3).normalize();
    const tf::Vector3	ry(0, 1, 0);
    const auto		rz = rx.cross(ry).normalize();

  // Paint motion mask over the input RGB image.
    for (int v = 0; v < mask.rows; ++v)
    {
	auto	p = mask.ptr<uint8_t>(v, 0);
	auto	q = image.ptr<cv::Vec3b>(v, 0);
	auto	r = depth.ptr<float>(v, 0);

	for (const auto pe = p + mask.cols; p < pe; ++p, ++q, ++r)
	    if (*p)
	    {
	      //(*q)[0] = 0;
		(*q)[1] = 196;
		(*q)[2] = 196;
	    }
	    else
		*r = 0;
    }

    cv::drawMarker(image, point_t(root), cv::Scalar(0, 255, 255),
		   cv::MARKER_CROSS, 20, 2);
    cv::drawMarker(image, point_t(cabletip), cv::Scalar(0, 255, 255),
		   cv::MARKER_CROSS, 20, 2);

    return tf::Transform({rx.x(), ry.x(), rz.x(),
			  rx.y(), ry.y(), rz.y(),
			  rx.z(), ry.z(), rz.z()},
			 tip3);
}

MotionDetector::vector3_t
MotionDetector::view_vector(const camera_info_cp& camera_info,
			    const value_t u, value_t v)
{
    cv::Mat_<value_t>	K(3, 3);
    std::copy_n(std::begin(camera_info->K), 9, K.begin());
    cv::Mat_<value_t>	D(1, 4);
    std::copy_n(std::begin(camera_info->D), 4, D.begin());
    cv::Mat_<point2_t>	uv(1, 1), xy(1, 1);
    uv(0) = {u, v};
    cv::undistortPoints(uv, xy, K, D);

    return {xy(0).x, xy(0).y, value_t(1)};
}

}	// namespace aist_motion_detector
