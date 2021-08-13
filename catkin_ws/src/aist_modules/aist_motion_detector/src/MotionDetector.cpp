/*!
 *  \file	MotionDetector.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS node for tracking corners in 2D images
 */
#include "MotionDetector.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace aist_motion_detector
{
/************************************************************************
*  static functions							*
************************************************************************/
inline cv::Point3f
pointTFToCV(const tf::Point& p)
{
    return {p.x(), p.y(), p.z()};
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
     _search_width(_nh.param("search_width", 0.040)),
     _search_height(_nh.param("search_height", 0.030))
{
  // Setup DetectMotion action server.
    _detect_motion_srv.registerGoalCallback(boost::bind(&goal_cb, this));
    _detect_motion_srv.registerPreemptCallback(boost::bind(&preempt_cb, this));
    _detect_motion_srv.start();

  // Setup callback for synced camera_info and depth.
    _sync.registerCallback(&image_cb, this);

  // Setup parameters and ddynamic_reconfigure server.
    _ddr.registerVariable<double>("search_width", &_search_width,
				  "Width of search area in meters",
				  0.005, 0.08);
    _ddr.registerVariable<double>("search_height", &_search_height,
				  "Height of search area in meters",
				  0.005, 0.08);
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
    ROS_INFO_STREAM("(MotionDetector) Given a goal["
		    << _current_goal->target_frame << ']');
}

void
MotionDetector::preempt_cb()
{
    _detect_motion_srv.setPreempted();
    ROS_INFO_STREAM("(MotionDetector) Cancelled a goal");
}

void
MotionDetector::image_cb(const camera_info_cp& camera_info,
			 const image_cp& image, const image_cp& depth)
{
    using namespace	sensor_msgs;

    using point2_t = cv::Point2f;
    using point3_t = cv::Point3f;
    using point_t  = cv::Point;

    if (!_detect_motion_srv.isActive())
    {
    	return;
    }

  // Project target position onto the image.
    try
    {
	const auto cv_img = cv_bridge::toCvShare(image, image_encodings::RGB8);
	
      // Update the background model.
	cv_bridge::CvImage	cv_mask;
	_bgsub->apply(cv_img->image, cv_mask.image);

	set_roi(cv_mask.image, _current_goal->target_frame, camera_info);

      // Publishe masked ROI.
	cv_mask.encoding = "mono8";
	cv_mask.header   = image->header;
	_image_pub.publish(cv_mask.toImageMsg());
    }
    catch (const tf::TransformException& err)
    {
	ROS_ERROR_STREAM("(MotionDetector) TransformException: "
			 << err.what());
	return;
    }
    catch (const cv_bridge::Exception& err)
    {
	ROS_ERROR_STREAM("(MotionDetector) cv_bridge exception: "
			 << err.what());
	return;
    }
}

void
MotionDetector::set_roi(cv::Mat& image, const std::string& target_frame,
			const camera_info_cp& camera_info) const
{
    using point2_t = cv::Point2f;
    using point3_t = cv::Point3f;
    using point_t  = cv::Point;

  // Get transform from the target frame to camera frame.
    _listener.waitForTransform(camera_info->header.frame_id, target_frame,
			       camera_info->header.stamp, ros::Duration(1.0));
    tf::StampedTransform	Tct;
    _listener.lookupTransform(camera_info->header.frame_id, target_frame,
			      camera_info->header.stamp, Tct);

  // Transfrom mask corners to camera frame.
    const tf::Point	corners[] = {{0.0,	      0.0, -_search_width/2},
				     {0.0,	      0.0,  _search_width/2},
				     {_search_height, 0.0,  _search_width/2},
				     {_search_height, 0.0, -_search_width/2}};
    cv::Mat_<point3_t>	pt3s(1, 4);
    for (size_t i = 0; i < 4; ++i)
	pt3s(i) = pointTFToCV(Tct * corners[i]);
    
  // Project mask corners onto the image.
    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info->K), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info->D), 4, D.begin());
    const auto		zero = cv::Mat_<float>::zeros(1, 3);
    cv::Mat_<point2_t>	pt2s(1, 4);
    cv::projectPoints(pt3s, zero, zero, K, D, pt2s);

  // Mask 
    const point_t	outer[]	  = {{0,	    0},
				     {image.cols-1, 0},
				     {image.cols-1, image.rows-1},
				     {0,	    image.rows-1}};
    const point_t	inner[]	  = {pt2s(0, 0), pt2s(0, 1),
				     pt2s(0, 2), pt2s(0, 3)};
    const point_t*	borders[] = {outer, inner};
    const int		npoints[] = {4, 4};
    cv::fillPoly(image, borders, npoints, 2, cv::Scalar(0), cv::LINE_8);
}
    
}	// namespace aist_motion_detector
