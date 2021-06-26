/*!
 *  \file	Localization.cpp
 *  \author	Toshio UESHIBA
 *  \brief	Server for finding and localizing top of the part box
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

  // Convert the given base plane to depth frame.
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

  // Set local transformation from model frame to gaze point frame.
    const tf::Transform	Tgm(tf::Matrix3x3(_current_goal->axes[0],
					  _current_goal->axes[1],
					  _current_goal->axes[2],
					  _current_goal->axes[3],
					  _current_goal->axes[4],
					  _current_goal->axes[5],
					  _current_goal->axes[6],
					  _current_goal->axes[7],
					  _current_goal->axes[8]),
			    tf::Vector3(_current_goal->offset[0],
					_current_goal->offset[1],
					_current_goal->offset[2]));
    result_t		result;
    result.poses.header = depth->header;

    value_t	min_error = std::numeric_limits<value_t>::max();
    for (const auto& pose2d : _current_goal->poses2d)
    {
	const auto	v = view_vector(camera_info,
					(pose2d.x > 0.0 ? pose2d.x :
					 0.5*camera_info->width),
					(pose2d.y > 0.0 ? pose2d.y :
					 0.5*camera_info->height));
	const auto	x = (-d/n.dot(v)) * v;
	auto		r = n.cross(vector3_t(std::cos(pose2d.theta),
					      std::sin(pose2d.theta), 0));
	r *= (value_t(1)/norm(r));
	const auto	q = r.cross(n);

      // Transformation: camera_frmae <== gaze_point_frame <== model_frame
	auto	Tcm = tf::Transform(tf::Matrix3x3(q(0), r(0), n(0),
						  q(1), r(1), n(1),
						  q(2), r(2), n(2)),
				    tf::Vector3(x(0), x(1), x(2))) * Tgm;

	if (_current_goal->refine_transform)
	{
	    value_t	error;

	    try
	    {
		Tcm = refine_transform(_current_goal->object_name,
				       Tcm, camera_info, depth, error);
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

		min_error = error;
	    }
	}
	else
	{
	    geometry_msgs::Pose	pose;
	    tf::poseTFToMsg(Tcm, pose);
	    result.poses.poses.push_back(pose);
	}
    }

    _localize_srv.setSucceeded(result);

    ROS_INFO_STREAM("(Localization) Succeeded: "
		    << result.poses.poses.size() << " pose(s) found.");
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
			       const image_cp& depth, value_t& error) const
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

  // Set inputs and parameters for ICP.
    icp_t	icp;
    icp.setInputSource(model_cloud);
    icp.setInputTarget(data_cloud);
    icp.setMaximumIterations(_niterations);
    icp.setMaxCorrespondenceDistance(_max_distance);
    icp.setTransformationEpsilon(_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(_fitness_epsilon);

  // Setup transformation from PCD cloud to URDF model. This corresponds to
  // the rotation from STL mesh to assy_part_XX link specified by
  // rpy="1.5707 0 0"
  // in o2ac_parts_description/urdf/generated/XX_YYYY_macro.urdf.xacro.
    const tf::Transform	Tmp({1.0, 0.0,  0.0,
			     0.0, 0.0, -1.0,
			     0.0, 1.0,  0.0},
			    {0.0, 0.0,  0.0});

  // Perform ICP to refine model pose
    pcl_cloud_p	registered_cloud(new pcl_cloud_t);
    icp.align(*registered_cloud, matrix44<value_t>(Tcm * Tmp));

    if (!icp.hasConverged())
	throw std::runtime_error("convergence failure in ICP");

  // Show registered model cloud.
    cloud_t	registered_cloud_msg;
    pcl::toROSMsg(*registered_cloud, registered_cloud_msg);
    _model_cloud_pub.publish(registered_cloud_msg);

  // Get transformation from model PCD cloud to camera frame.
    error = icp.getFitnessScore();
    const auto	Tcp = icp.getFinalTransformation();

  // Return transformation from URDF model to camera frame.
    return tf::Transform({Tcp(0, 0), Tcp(0, 1), Tcp(0, 2),
    			  Tcp(1, 0), Tcp(1, 1), Tcp(1, 2),
    			  Tcp(2, 0), Tcp(2, 1), Tcp(2, 2)},
    			 {Tcp(0, 3), Tcp(1, 3), Tcp(2, 3)}) * Tmp.inverse();
}

}	// namespace aist_new_localization
