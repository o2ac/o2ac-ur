/*!
 *  \file	MotionDetector.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS node for tracking corners in 2D images
 */
#include "MotionDetector.h"
#include "Plane.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/bgsegm.hpp>

namespace aist_motion_detector
{
/************************************************************************
*  static functions							*
************************************************************************/
static inline cv::Point3f
pointTFToCV(const tf::Point& p)
{
    return {p.x(), p.y(), p.z()};
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
     _listener(),
     _detect_motion_srv(_nh, "detect_motion", false),
     _current_goal(nullptr),
     _ddr(_nh),
     _bgsub(cv::createBackgroundSubtractorMOG2()),
     _search_top(_nh.param("search_top", 0.003)),
     _search_bottom(_nh.param("search_bottom", 0.030)),
     _search_width(_nh.param("search_width", 0.050)),
     _nframes(0),
     _top_left(0, 0),
     _bottom_right(0, 0),
     _corners(),
     _mask()
{
  // Setup DetectMotion action server.
    _detect_motion_srv.registerGoalCallback(boost::bind(&goal_cb, this));
    _detect_motion_srv.registerPreemptCallback(boost::bind(&preempt_cb, this));
    _detect_motion_srv.start();

  // Setup callback for synced camera_info and depth.
    _sync.registerCallback(&image_cb, this);

  // Setup parameters and ddynamic_reconfigure server.
    _ddr.registerVariable<double>("search_top", &_search_top,
				  "Top of search area in meters",
				  -0.005, 0.005);
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
    _current_goal = _detect_motion_srv.acceptNewGoal();
    _nframes	  = 0;
    ROS_INFO_STREAM("(MotionDetector) Given a goal["
		    << _current_goal->target_frame << ']');
}

void
MotionDetector::preempt_cb()
{
    if (_nframes > 0)
    {
	detect_cable_tip();
	_image_pub.publish(_mask.toImageMsg());

	DetectMotionResult	result;
	_detect_motion_srv.setPreempted(result);

	_nframes = 0;
    }
    else
	_detect_motion_srv.setPreempted();
    ROS_INFO_STREAM("(MotionDetector) Cancelled a goal");
}

void
MotionDetector::image_cb(const camera_info_cp& camera_info,
			 const image_cp& image, const image_cp& depth)
{
    if (!_detect_motion_srv.isActive())
    	return;

    try
    {
      // Perform background subtraction
	cv_bridge::CvImage	mask;
	_bgsub->apply(cv_bridge::toCvShare(image)->image, mask.image);

      // Accumulate updated foreground.
	accumulate_mask(mask.image, _current_goal->target_frame, camera_info);

	mask.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
	mask.header   = camera_info->header;
	_image_pub.publish(mask.toImageMsg());
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

void
MotionDetector::accumulate_mask(const cv::Mat& mask,
				const std::string& target_frame,
				const camera_info_cp& camera_info)
{
  // Get transform from the target frame to camera frame.
    _listener.waitForTransform(camera_info->header.frame_id, target_frame,
			       camera_info->header.stamp, ros::Duration(1.0));
    tf::StampedTransform	Tct;
    _listener.lookupTransform(camera_info->header.frame_id, target_frame,
			      camera_info->header.stamp, Tct);

  // Transfrom ROI corners to camera frame.
    const tf::Point	corners[] = {{_search_top,    0.0, -_search_width/2},
				     {_search_top,    0.0,  _search_width/2},
				     {_search_bottom, 0.0,  _search_width/2},
				     {_search_bottom, 0.0, -_search_width/2}};
    cv::Mat_<point3_t>	pt3s(1, 4);
    for (size_t i = 0; i < 4; ++i)
	pt3s(i) = pointTFToCV(Tct * corners[i]);

  // Project ROI corners onto the image.
    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info->K), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info->D), 4, D.begin());
    const auto		zero = cv::Mat_<float>::zeros(1, 3);
    cv::Mat_<point2_t>	p(1, 4);
    cv::projectPoints(pt3s, zero, zero, K, D, p);

  // Confirm that all the projected corners are included within the image.
    if (!withinImage(p(0), mask) || !withinImage(p(1), mask) ||
	!withinImage(p(2), mask) || !withinImage(p(3), mask))
	return;

    _mask.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    _mask.header   = camera_info->header;

    if (_nframes == 0)
    {
      // Crop the given image.
	_top_left.x	= int(std::min({p(0).x, p(1).x, p(2).x, p(3).x}));
	_top_left.y	= int(std::min({p(0).y, p(1).y, p(2).y, p(3).y}));
	_bottom_right.x = int(std::max({p(0).x, p(1).x, p(2).x, p(3).x}));
	_bottom_right.y	= int(std::max({p(0).y, p(1).y, p(2).y, p(3).y}));
	_corners	= p;
	mask(cv::Rect(_top_left, _bottom_right)).convertTo(_mask.image,
							   CV_16UC1);
    }
    else
    {
	cv::Mat	warped_mask;
	cv::warpPerspective(mask, warped_mask, cv::findHomography(_corners, p),
			    mask.size(), cv::WARP_INVERSE_MAP);
	cv::Mat	warped_and_converted_mask;
	warped_mask(cv::Rect(_top_left, _bottom_right))
	    .convertTo(warped_and_converted_mask, CV_16UC1);
	_mask.image += warped_and_converted_mask;
    }

    ++_nframes;
}

void
MotionDetector::detect_cable_tip()
{
  // Fill the outside of ROI with zero.
    const point_t	outer[]   = {{0,		  0},
				     {_mask.image.cols-1, 0},
				     {_mask.image.cols-1, _mask.image.rows-1},
				     {0,		  _mask.image.rows-1}};
    const point_t	inner[]   = {point_t(_corners(0)) - _top_left,
				     point_t(_corners(1)) - _top_left,
				     point_t(_corners(2)) - _top_left,
				     point_t(_corners(3)) - _top_left};
    const point_t*	borders[] = {outer, inner};
    const int		npoints[] = {4, 4};
    cv::fillPoly(_mask.image, borders, npoints, 2, cv::Scalar(0), cv::LINE_8);

  // Binarize the accumultated mask image.
    _mask.image /= _nframes;
    cv::Mat	mask;
    _mask.image.convertTo(mask, CV_8UC1);
    cv::threshold(mask, _mask.image, 0, 255,
		  cv::THRESH_BINARY | cv::THRESH_OTSU);
    _mask.encoding = sensor_msgs::image_encodings::TYPE_8UC1;

  // Create a line of finger-tip border.
    const cv::Vec<float, 2>	ends[] = {_corners(0) - point2_t(_top_left),
					  _corners(1) - point2_t(_top_left)};
    const TU::Plane<float, 2>	finger_tip(ends, ends + 2);

  // Assign labels to the binarized mask image.
    cv::Mat	labels, stats, centroids;
    const auto	nlabels = cv::connectedComponentsWithStats(mask, labels,
							   stats, centroids);

  // Find a largest region close to the finger-tip border.
    const auto	lmax = findLargestRegion(labels, stats, nlabels, finger_tip);
    
  // Fit a line to the points in the region.
    std::vector<cv::Vec<float, 2> >	points;
    for (int v = 0; v < labels.rows; ++v)
    {
	auto	p = labels.ptr<int>(v, 0);
	auto	q = _mask.image.ptr<uint8_t>(v, 0);

	for (int u = 0; u < labels.cols; ++u, ++p, ++q)
	    if (*p == lmax)
	    {
		points.push_back({u, v});
		*q = 255;
	    }
	    else
		*q = 0;
    }
    TU::Plane<float, 2>	cable(points.begin(), points.end());

    for (const auto& point : points)
    {
	const auto	p = cable.projection(point);

    }
}

}	// namespace aist_motion_detector
