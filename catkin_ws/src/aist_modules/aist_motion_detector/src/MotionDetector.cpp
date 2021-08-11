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
     _search_width(_nh.param("search_width", 150)),
     _search_height(_nh.param("search_height", 150)),
     _search_offset(_nh.param("search_offset", 50))
{
  // Setup DetectMotion action server.
    _detect_motion_srv.registerGoalCallback(boost::bind(&goal_cb, this));
    _detect_motion_srv.registerPreemptCallback(boost::bind(&preempt_cb, this));
    _detect_motion_srv.start();

  // Setup callback for synced camera_info and depth.
    _sync.registerCallback(&image_cb, this);

  // Setup parameters and ddynamic_reconfigure server.
    _ddr.registerVariable<int>("search_width", &_search_width,
			       "Width of search area in pixels", 10, 250);
    _ddr.registerVariable<int>("search_height", &_search_height,
			       "Height of search area in pixels", 10, 250);
    _ddr.registerVariable<int>("search_offset", &_search_offset,
			       "Height of search area in pixels", 0, 150);
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
		    << _current_goal->target.header.frame_id << ']');
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
    cv::Point			p;
    cv_bridge::CvImageConstPtr	cv_img;
    try
    {
	const auto pt2 = project_point(_current_goal->target, camera_info);
	p.x = int(pt2.x + 0.5f);
	p.y = int(pt2.y + 0.5f);
	cv_img = cv_bridge::toCvShare(image, image_encodings::RGB8);
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

  // Update the background model.
    cv::Mat	mask;
    _bgsub->apply(cv_img->image, mask);

    cv::Point	top_left;
    p.y += _search_offset;
    const auto	roi = create_roi(mask, p, top_left);
    
    cv_bridge::CvImage	cv_mask;
    cv::cvtColor(mask, cv_mask.image, cv::COLOR_GRAY2RGB);
    cv::drawMarker(cv_mask.image, p, cv::Scalar(255, 0, 0));
    const cv::Point	corners[]
			    = {{top_left.x,	       top_left.y},
			       {top_left.x+roi.cols-1, top_left.y},
			       {top_left.x+roi.cols-1, top_left.y+roi.rows-1},
			       {top_left.x,	       top_left.y+roi.rows-1}};
    const cv::Point*	contour[] = {corners};
    const int		npoints = 4;
    cv::polylines(cv_mask.image, contour, &npoints, 1, true,
		  cv::Scalar(255, 0, 0), 4);
    cv_mask.encoding = "rgb8";
    cv_mask.header = image->header;
    _image_pub.publish(cv_mask.toImageMsg());
}

cv::Point2f
MotionDetector::project_point(const geometry_msgs::PointStamped& point,
			      const camera_info_cp& camera_info) const
{
  // Transform the given point to camera frame.
    _listener.waitForTransform(camera_info->header.frame_id,
			       point.header.frame_id,
			       camera_info->header.stamp, ros::Duration(1.0));
    tf::Stamped<tf::Point>	p;
    pointStampedMsgToTF(point, p);
    p.stamp_ = camera_info->header.stamp;
    _listener.transformPoint(camera_info->header.frame_id, p, p);

  // Convert the transformed point to OpenCV format.
    cv::Mat_<cv::Point3f>	pt3(1, 1);
    pt3(0) = pointTFToCV(p);

  // Project inner boundary vertices onto image.
    cv::Mat_<float>		K(3, 3);
    std::copy_n(std::begin(camera_info->K), 9, K.begin());
    cv::Mat_<float>		D(1, 4);
    std::copy_n(std::begin(camera_info->D), 4, D.begin());
    const auto		zero = cv::Mat_<float>::zeros(1, 3);
    cv::Mat_<cv::Point2f>	pt2(1, 1);
    cv::projectPoints(pt3, zero, zero, K, D, pt2);

    return pt2(0);
}

cv::Mat
MotionDetector::create_roi(const cv::Mat& image,
			   const cv::Point& center, cv::Point& top_left) const
{
    top_left.x = std::max(center.x - _search_width/2,  0);
    top_left.y = std::max(center.y - _search_height/2, 0);
    return image(cv::Rect(top_left.x, top_left.y,
			  std::min(_search_width,  image.cols - top_left.x),
			  std::min(_search_height, image.rows - top_left.y)));
}
    
    
}	// namespace aist_motion_detector
