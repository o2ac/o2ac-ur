/*!
 *  \file	MotionDetector.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS node for tracking corners in 2D images
 */
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "MotionDetector.h"

namespace aist_motion_detector
{
/************************************************************************
*  static functions							*
************************************************************************/

/************************************************************************
*  class MotionDetector							*
************************************************************************/
MotionDetector::MotionDetector(const ros::NodeHandle& nh)
    :_nh(nh),
     _select_srv(_nh.advertiseService("select", &select_cb, this)),
     _it(_nh),
     _camera_info_sub(_nh, "/camera_info", 1),
     _image_sub(_it, "/image", 1),
     _depth_sub(_it, "/depth", 1),
     _sync(sync_policy_t(10), _camera_info_sub, _image_sub, _depth_sub),
     _camera_pub(_it.advertiseCamera("depth", 1)),
     _image_pub(_it.advertise("image", 1)),
     _ddr(_nh)
{
  // Setup callback for synced camera_info and depth.
    _sync.registerCallback(&image_cb, this);
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
MotionDetector::image_cb(const camera_info_cp& camera_info,
			 const image_cp& image, const image_cp& depth)
{
}

void
MotionDetector::goal_cb()
{
  //_current_goal = _detect_motion_srv.acceptNewGoal();
    ROS_INFO_STREAM("(motion_detector) Given a goal");
}

void
MotionDetector::preempt_cb()
{
  //_detect_motion_srv.setPreempted();
    ROS_INFO_STREAM("(motion_detector) Cancelled a goal");
}

}	// namespace aist_motion_detector
