/*!
 *  \file	MotionDetector.cpp
 *  \author	Toshio UESHIBA
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
     _camera_pub(_it.advertiseCamera("depth", 1)),
     _image_pub(_it.advertise("image", 1)),
     _mask_pub(_it.advertise("mask", 1)),
     _listener(),
     _broadcaster(),
     _find_cabletip_srv(_nh, "find_cabletip", false),
     _current_goal(nullptr),
     _ddr(_nh),
     _bgsub(cv::createBackgroundSubtractorMOG2()),
     _search_top(_nh.param("search_top", 0.005)),
     _search_bottom(_nh.param("search_bottom", 0.030)),
     _search_width(_nh.param("search_width", 0.050)),
     _nframes(0),
     _top_left(0, 0),
     _corners(),
     _cv_image(nullptr),
     _cv_accum(),
     _Tct(),
     _camera_info(nullptr)
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
    _nframes	  = 0;
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
    _cv_image = cv_bridge::toCvCopy(image);
    
    if (_find_cabletip_srv.isActive())
    {
	try
	{
	  // Perform background subtraction
	    cv_bridge::CvImage	cv_mask;
	    cv_mask.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
	    cv_mask.header   = camera_info->header;
	    _bgsub->apply(_cv_image->image, cv_mask.image);

	  // Binarize mask image.
	    for (int v = 0; v < cv_mask.image.rows; ++v)
	    {
		auto	p = cv_mask.image.ptr<uint8_t>(v, 0);

		for (const auto pe = p + _cv_accum.image.cols; p < pe; ++p)
		    if (*p < 255)
			*p = 0;
	    }

	  // Accumulate updated foreground.
	    accumulate_mask(cv_mask.image,
			    _current_goal->target_frame, camera_info);
	    _mask_pub.publish(cv_mask.toImageMsg());
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
    }

    _image_pub.publish(_cv_image->toImageMsg());
}

void
MotionDetector::accumulate_mask(const cv::Mat& mask,
				const std::string& target_frame,
				const camera_info_cp& camera_info)
{
    _camera_info = camera_info;
    
  // Get transform from the target frame to camera frame.
    _listener.waitForTransform(_camera_info->header.frame_id, target_frame,
			       _camera_info->header.stamp, ros::Duration(1.0));
    _listener.lookupTransform(_camera_info->header.frame_id, target_frame,
			      _camera_info->header.stamp, _Tct);

  // Transfrom ROI corners to camera frame.
    const tf::Point	corners[] = {{_search_top,    0.0, -_search_width/2},
				     {_search_top,    0.0,  _search_width/2},
				     {_search_bottom, 0.0,  _search_width/2},
				     {_search_bottom, 0.0, -_search_width/2}};
    cv::Mat_<point3_t>	pt3s(1, 4);
    for (size_t i = 0; i < 4; ++i)
	pt3s(i) = pointTFToCV<value_t>(_Tct * corners[i]);

  // Project ROI corners onto the image.
    cv::Mat_<value_t>	K(3, 3);
    std::copy_n(std::begin(_camera_info->K), 9, K.begin());
    cv::Mat_<value_t>	D(1, 4);
    std::copy_n(std::begin(_camera_info->D), 4, D.begin());
    const auto		zero = cv::Mat_<value_t>::zeros(1, 3);
    cv::Mat_<point2_t>	p(1, 4);
    cv::projectPoints(pt3s, zero, zero, K, D, p);

  // Confirm that all the projected corners are included within the image.
    if (!withinImage(p(0), mask) || !withinImage(p(1), mask) ||
	!withinImage(p(2), mask) || !withinImage(p(3), mask))
	return;

    const point_t top_left(int(std::min({p(0).x, p(1).x, p(2).x, p(3).x})),
			   int(std::min({p(0).y, p(1).y, p(2).y, p(3).y})));

    if (_nframes++ == 0)
    {
      // Crop the given image.
	const point_t	bottom_right(
			    int(std::max({p(0).x, p(1).x, p(2).x, p(3).x})),
			    int(std::max({p(0).y, p(1).y, p(2).y, p(3).y})));
	mask(cv::Rect(top_left, bottom_right)).convertTo(_cv_accum.image,
							 CV_16UC1);
	_cv_accum.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	_cv_accum.header   = _camera_info->header;
	_top_left	  = top_left;
	_corners	  = p;
    }
    else
    {
	cv::Mat	warped_mask;
	cv::warpPerspective(mask, warped_mask, cv::findHomography(_corners, p),
			    mask.size(), cv::WARP_INVERSE_MAP);
	cv::Mat	warped_and_converted_mask;
	warped_mask(cv::Rect(_top_left, _cv_accum.image.size()))
	    .convertTo(warped_and_converted_mask, CV_16UC1);
	(_cv_accum.image += warped_and_converted_mask) /= 2;
    }

  // Paint motion mask over the input RGB image.
    for (int v = 0; v < _cv_accum.image.rows; ++v)
    {
	auto	p = _cv_accum.image.ptr<uint16_t>(v, 0);
	auto	q = _cv_image->image.ptr<cv::Vec3b>(top_left.y + v, 0)
		  + top_left.x;

	for (const auto pe = p + _cv_accum.image.cols; p < pe; ++p, ++q)
	    if (*p >= 255)
	    {
		(*q)[1] = 255;
		(*q)[2] = 255;
	    }
    }

    const auto	T = find_cabletip(top_left);
    _broadcaster.sendTransform({T, ros::Time::now(),
				_current_goal->target_frame,
				"cabletip_link"});
}

tf::Transform
MotionDetector::find_cabletip(const point_t& top_left)
{
  // Fill the outside of ROI with zero.
    const point_t	outer[] =
			{{0,			  0},
			 {_cv_accum.image.cols-1, 0},
			 {_cv_accum.image.cols-1, _cv_accum.image.rows-1},
			 {0,			  _cv_accum.image.rows-1}};
    const point_t	inner[]   = {point_t(_corners(0)) - _top_left,
				     point_t(_corners(1)) - _top_left,
				     point_t(_corners(2)) - _top_left,
				     point_t(_corners(3)) - _top_left};
    const point_t*	borders[] = {outer, inner};
    const int		npoints[] = {4, 4};
    cv::fillPoly(_cv_accum.image, borders, npoints, 2,
		 cv::Scalar(0), cv::LINE_8);

  // Binarize the accumultated mask image.
  //_cv_accum.image /= _nframes;
    cv::Mat	mask;
    _cv_accum.image.convertTo(mask, CV_8UC1);
    cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // Create a line of finger-tip border.
    const cv::Vec<value_t, 2>	ends[] = {_corners(0) - point2_t(_top_left),
					  _corners(1) - point2_t(_top_left)};
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

    cv::drawMarker(mask, point_t(root),     cv::Scalar(128));
    cv::drawMarker(mask, point_t(cabletip), cv::Scalar(128));
    
  // Plane including fingertip described w.r.t. camera frame.
    const auto		nc = vector3TFToCV<value_t>(_Tct.getBasis()
						    .getColumn(1));
    const auto		tc = vector3TFToCV<value_t>(_Tct.getOrigin());
    const plane_t	plane(nc, -nc.dot(tc));

  // Compute cross point between the plane and view vectors
  // for root and fingertip.
    const auto		root3 = _Tct.inverse()
			      * vector3CVToTF(
				  plane.cross_point(
				      view_vector(root(0) + _top_left.x,
						  root(1) + _top_left.y)));
    const auto		tip3  = _Tct.inverse()
			      * vector3CVToTF(
				  plane.cross_point(
				      view_vector(cabletip.x + _top_left.x,
						  cabletip.y + _top_left.y)));
    const auto		rx = (tip3 - root3).normalize();
    const tf::Vector3	ry(0, 1, 0);
    const auto		rz = rx.cross(ry).normalize();

    std::cerr << "root3 = " << root3 << std::endl;
    std::cerr << "tip3  = " << tip3  << std::endl;
    
  // Paint motion mask over the input RGB image.
    for (int v = 0; v < mask.rows; ++v)
    {
	auto	p = mask.ptr<uint8_t>(v, 0);
	auto	q = _cv_image->image.ptr<cv::Vec3b>(top_left.y + v, 0)
		  + top_left.x;

	for (const auto pe = p + mask.cols; p < pe; ++p, ++q)
	    if (*p)
	    {
	      //(*q)[0] = 0;
		(*q)[1] = 255;
	      //(*q)[2] = 0;
	    }
    }

    return tf::Transform({rx.x(), ry.x(), rz.x(),
			  rx.y(), ry.y(), rz.y(),
			  rx.z(), ry.z(), rz.z()},
			 tip3);
}

MotionDetector::vector3_t
MotionDetector::view_vector(const value_t u, value_t v) const
{
    cv::Mat_<value_t>	K(3, 3);
    std::copy_n(std::begin(_camera_info->K), 9, K.begin());
    cv::Mat_<value_t>	D(1, 4);
    std::copy_n(std::begin(_camera_info->D), 4, D.begin());
    cv::Mat_<point2_t>	uv(1, 1), xy(1, 1);
    uv(0) = {u, v};
    cv::undistortPoints(uv, xy, K, D);

    return {xy(0).x, xy(0).y, value_t(1)};
}

}	// namespace aist_motion_detector
