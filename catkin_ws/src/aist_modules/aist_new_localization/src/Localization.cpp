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
 *  \file	Localization.cpp
 *  \author	Toshio Ueshiba
 *  \brief	Server for finding and localizing parts on the tray
 */
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include "Localization.h"
#include "utils.h"

namespace aist_new_localization
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> static Eigen::Matrix<T, 4, 4>
matrix44(const tf::Transform& transform)
{
    const auto&			R = transform.getBasis();
    const auto&			t = transform.getOrigin();
    Eigen::Matrix<T, 4, 4>	m;
    m << R[0][0], R[0][1], R[0][2], t[0],
	 R[1][0], R[1][1], R[1][2], t[1],
	 R[2][0], R[2][1], R[2][2], t[2],
	    T(0),    T(0),    T(0), T(1);
    return m;
}

cv::Point3f
pclpointToCV(const pcl::PointXYZ& p)
{
    return {p.x, p.y, p.z};
}

template <class T, unsigned long int D> static std::ostream&
operator <<(std::ostream& out, const boost::array<T, D>& vec)
{
    for (const auto& val : vec)
	out << ' ' << val;
    return out << std::endl;
}

static std::ostream&
operator <<(std::ostream& out, const tf::Vector3& v)
{
    return out << ' ' << v.x() << ' ' << v.y() << ' ' << v.z() << std::endl;
}

static std::ostream&
operator <<(std::ostream& out, const tf::Matrix3x3& m)
{
    return out << m.getRow(0) << m.getRow(1) << m.getRow(2);
}

static std::ostream&
operator <<(std::ostream& out, const tf::Quaternion& q)
{
    return out << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
	       << std::endl;
}

static std::ostream&
operator <<(std::ostream& out, const tf::Transform& transform)
{
    return out << "origin:" << transform.getOrigin()
	       << "rotation:\n" << transform.getBasis();
}

static std::ostream&
operator <<(std::ostream& out, const aist_depth_filter::PlaneStamped& plane)
{
    return out << "frame_id: " << plane.header.frame_id << '\n' << plane.plane;
}

/************************************************************************
*  class Localization							*
************************************************************************/
Localization::Localization(const ros::NodeHandle& nh)
    :_nh(nh),
     _listener(),
     _camera_info_sub(_nh, "/camera_info", 1),
     _depth_sub(_nh, "/depth",  1),
     _sync(sync_policy_t(10), _camera_info_sub, _depth_sub),
     _model_cloud_pub(_nh.advertise<cloud_t>("model_points", 1)),
     _data_cloud_pub(_nh.advertise<cloud_t>("data_points", 1)),
     _localize_srv(_nh, "localize", false),
     _current_goal(nullptr),
     _ddr(_nh),
     _pcd_dir(_nh.param("pcd_dir",
			ros::package::getPath("o2ac_parts_description")
			+ "/meshes")),
     _niterations(_nh.param("niterations", 1000000)),
     _max_distance(_nh.param("max_distance", 0.002)),
     _transformation_epsilon(1.0e-13),
     _fitness_epsilon(1.0e-13)
{
  // Setup callback for synced camera_info and depth
    _sync.registerCallback(&Localization::localize_cb, this);

  // Setup Localize action server.
    _localize_srv.registerGoalCallback(boost::bind(&goal_cb, this));
    _localize_srv.registerPreemptCallback(boost::bind(&preempt_cb, this));
    _localize_srv.start();

  // Setup parameters and ddynamic_reconfigure server.
    _ddr.registerVariable<int>("niterations", &_niterations,
			       "Max. number of iterations", 1000, 1000000);
    _ddr.registerVariable<double>("max_distance", &_max_distance,
				  "Max correspondence distance", 0.001, 0.1);
    _ddr.publishServicesTopics();
}

void
Localization::run()
{
    ros::spin();
}

void
Localization::goal_cb()
{
    _current_goal = _localize_srv.acceptNewGoal();
    ROS_INFO_STREAM("(Localization) Given a goal["
		    << _current_goal->object_name << ']');
}

void
Localization::preempt_cb()
{
    _localize_srv.setPreempted();
    ROS_INFO_STREAM("(Localization) Cancelled a goal");
}

void
Localization::localize_cb(const camera_info_cp& camera_info,
			  const image_cp& depth)
{
    if (!_localize_srv.isActive())
	return;

    ROS_INFO_STREAM("(Localization) Activated a goal["
		    << _current_goal->object_name << "]...");

  // Convert the given base plane to camera frame.
    vector3_t	n;
    value_t	d;
    try
    {
	transform_plane(depth->header.frame_id, _current_goal->plane, n, d);
    }
    catch (const tf::TransformException& e)
    {
	ROS_ERROR_STREAM("(Localization) Error in transform_plane(): "
			 << e.what());
	_localize_srv.setAborted();
	return;
    }

  // Transformation: gaze_point_frame <== URDF model_frame
    const tf::Transform	Tgm({_current_goal->origin.orientation.x,
			     _current_goal->origin.orientation.y,
			     _current_goal->origin.orientation.z,
			     _current_goal->origin.orientation.w},
			    {_current_goal->origin.position.x,
			     _current_goal->origin.position.y,
			     _current_goal->origin.position.z});
    result_t		result;
    result.poses.header = depth->header;

    value_t	min_error = std::numeric_limits<value_t>::max();
    for (const auto& pose2d : _current_goal->poses2d)
    {
	const auto	v = view_vector(camera_info, pose2d.x, pose2d.y);
	const auto	x = (-d/n.dot(v)) * v;
	auto		r = n.cross(vector3_t(std::cos(pose2d.theta),
					      std::sin(pose2d.theta), 0));
	r *= (value_t(1)/norm(r));
	const auto	q = r.cross(n);

      // Transformation: camera_frame <== gaze_point_frame <== model_frame
	auto		Tcm = tf::Transform({q(0), r(0), n(0),
					     q(1), r(1), n(1),
					     q(2), r(2), n(2)},
					    {x(0), x(1), x(2)}) * Tgm;

	if (_current_goal->refine_transform)
	{
	  // Check border
	    uint32_t	check_borders = 0x0;
	    if (_current_goal->check_borders)
	    {
		if (pose2d.x >= std::floor(0.5*depth->width))
		    check_borders |= CHECK_LEFT_BORDER;
		if (pose2d.x <= std::ceil(0.5*depth->width))
		    check_borders |= CHECK_RIGHT_BORDER;
		if (pose2d.y >= std::floor(0.5*depth->height))
		    check_borders |= CHECK_UPPER_BORDER;
		if (pose2d.y <= std::ceil(0.5*depth->height))
		    check_borders |= CHECK_LOWER_BORDER;
	    }

	    value_t	error;

	    try
	    {
		Tcm = refine_transform(_current_goal->object_name,
				       Tcm, camera_info, depth,
				       check_borders, error);
	    }
	    catch (const std::exception& err)
	    {
		_localize_srv.setAborted();

		ROS_ERROR_STREAM("(Localization) Aborted: failed to refine transform["
				 << err.what() << ']');
		return;
	    }

	    if (error < min_error)
	    {
		geometry_msgs::Pose	pose;
		tf::poseTFToMsg(Tcm, pose);
		result.poses.poses.resize(1);
		result.poses.poses[0] = pose;
		result.error	      = error;

		min_error = error;
	    }
	}
	else
	{
	    geometry_msgs::Pose	pose;
	    tf::poseTFToMsg(Tcm, pose);
	    result.poses.poses.push_back(pose);
	    result.error = 0;
	}
    }

    _localize_srv.setSucceeded(result);

    ROS_INFO_STREAM("(Localization) Succeeded: "
		    << result.poses.poses.size()
		    << " pose(s) found with error="
		    << result.error);
}

void
Localization::transform_plane(const std::string& target_frame,
			      const plane_t& plane,
			      vector3_t& normal, value_t& distance) const
{
    tf::StampedTransform	transform;
    _listener.waitForTransform(target_frame, plane.header.frame_id,
			       plane.header.stamp, ros::Duration(1.0));
    _listener.lookupTransform(target_frame, plane.header.frame_id,
			      plane.header.stamp, transform);

    const auto	n = transform.getBasis() * tf::Vector3(plane.plane.normal.x,
						       plane.plane.normal.y,
						       plane.plane.normal.z);
    normal(0) = n.x();
    normal(1) = n.y();
    normal(2) = n.z();
    distance  = plane.plane.distance - n.dot(transform.getOrigin());
}

Localization::vector3_t
Localization::view_vector(const camera_info_cp& camera_info,
			  value_t u, value_t v)
{
    using point2_t	= cv::Point_<value_t>;

    cv::Mat_<value_t>	K(3, 3);
    std::copy_n(std::begin(camera_info->K), 9, K.begin());
    cv::Mat_<value_t>	D(1, 4);
    std::copy_n(std::begin(camera_info->D), 4, D.begin());
    cv::Mat_<point2_t>	uv(1, 1), xy(1, 1);
    uv(0) = {u, v};
    cv::undistortPoints(uv, xy, K, D);

    return {xy(0).x, xy(0).y, value_t(1)};
}

tf::Transform
Localization::refine_transform(const std::string& object_name,
			       const tf::Transform& Tcm,
			       const camera_info_cp& camera_info,
			       const image_cp& depth, uint32_t check_borders,
			       value_t& error) const
{
    using namespace sensor_msgs;
    using pcl_point_t	= pcl::PointXYZ;
    using pcl_cloud_t	= pcl::PointCloud<pcl_point_t>;
    using pcl_cloud_p	= pcl_cloud_t::Ptr;
    using icp_t		= pcl::IterativeClosestPoint<pcl_point_t, pcl_point_t>;

  // Load model PCD and convert to PCL cloud.
    pcl_cloud_p	model_cloud(new pcl_cloud_t);
    const auto	pcd_file = _pcd_dir + '/' + object_name + ".pcd";
    if (pcl::io::loadPCDFile<pcl_point_t>(pcd_file, *model_cloud) < 0)
	throw std::runtime_error("cannot load model PCD: " + pcd_file);
    model_cloud->header.frame_id = depth->header.frame_id;
    model_cloud->header.stamp = pcl_conversions::toPCL(depth->header.stamp);

  // Convert depth image to PCL cloud.
    pcl_cloud_p	data_cloud(new pcl_cloud_t);
    if (depth->encoding == image_encodings::MONO16 ||
	depth->encoding == image_encodings::TYPE_16UC1)
    {
	pcl::fromROSMsg(create_pointcloud<uint16_t>(*camera_info, *depth),
			*data_cloud);
    }
    else if (depth->encoding == image_encodings::TYPE_32FC1)
    {
	pcl::fromROSMsg(create_pointcloud<float>(*camera_info, *depth),
			*data_cloud);
    }
    else
    {
	throw std::runtime_error("unknown dpeth encoding: " + depth->encoding);
    }

  // Show data cloud.
    cloud_t	data_cloud_msg;
    pcl::toROSMsg(*data_cloud, data_cloud_msg);
    _data_cloud_pub.publish(data_cloud_msg);

  // Confirm that model and data cloud are not empty.
    if (model_cloud->empty())
	throw std::runtime_error("empty model cloud!");
    if (data_cloud->empty())
	throw std::runtime_error("empty data cloud!");

  // Set inputs and parameters for ICP.
    icp_t	icp;
    icp.setInputSource(model_cloud);
    icp.setInputTarget(data_cloud);
    icp.setMaximumIterations(_niterations);
    icp.setMaxCorrespondenceDistance(_max_distance);
    icp.setTransformationEpsilon(_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(_fitness_epsilon);

  // Perform ICP to refine model pose
    pcl_cloud_p	registered_cloud(new pcl_cloud_t);
    icp.align(*registered_cloud, matrix44<value_t>(Tcm));

    if (!icp.hasConverged())
	throw std::runtime_error("convergence failure in ICP");

  // Show registered model cloud.
    cloud_t	registered_cloud_msg;
    pcl::toROSMsg(*registered_cloud, registered_cloud_msg);
    _model_cloud_pub.publish(registered_cloud_msg);

  // Check if registered model cloud is involved within the view volume.
    if (within_view_volume(registered_cloud->begin(), registered_cloud->end(),
			   camera_info, check_borders))
	error = icp.getFitnessScore();
    else
	error = std::numeric_limits<value_t>::max();

  // Return transformation from URDF model to camera frame.
    const auto	Tcp = icp.getFinalTransformation();
    return tf::Transform({Tcp(0, 0), Tcp(0, 1), Tcp(0, 2),
    			  Tcp(1, 0), Tcp(1, 1), Tcp(1, 2),
    			  Tcp(2, 0), Tcp(2, 1), Tcp(2, 2)},
    			 {Tcp(0, 3), Tcp(1, 3), Tcp(2, 3)});
}

template <class ITER> bool
Localization::within_view_volume(ITER begin, ITER end,
				 const camera_info_cp& camera_info,
				 uint32_t check_borders) const
{
    const auto	v0 = view_vector(camera_info, 0, 0);
    const auto	v1 = view_vector(camera_info, camera_info->width, 0);
    const auto	v2 = view_vector(camera_info, camera_info->width,
					      camera_info->height);
    const auto	v3 = view_vector(camera_info, 0, camera_info->height);
    std::array<vector3_t, 4>	normals = {v0.cross(v1), v1.cross(v2),
					   v2.cross(v3), v3.cross(v0)};

    uint32_t	mask = 0x1;
    for (const auto& normal : normals)
    {
	if (check_borders & mask)
	    for (auto point = begin; point != end; ++point)
		if (normal.dot(pclpointToCV(*point)) < 0)
		{
		    ROS_WARN_STREAM("(Localization) Collision against "
				    << (mask == CHECK_UPPER_BORDER ? "upper" :
					mask == CHECK_RIGHT_BORDER ? "right" :
					mask == CHECK_LOWER_BORDER ? "lower" :
								     "left")
				    << " border of bounding box");
		    return false;
		}
	mask <<= 1;
    }

    return true;
}

}	// namespace aist_new_localization
