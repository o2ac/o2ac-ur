/*!
 *  \file	nodelet.cpp
 *  \author	Toshio UESHIBA
 */
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "MotionDetector.h"

namespace aist_motion_detector
{
/************************************************************************
*  class MotionDetectorNodelet						*
************************************************************************/
class MotionDetectorNodelet : public nodelet::Nodelet
{
  public:
			MotionDetectorNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<MotionDetector>	_node;
};

void
MotionDetectorNodelet::onInit()
{
    NODELET_INFO("aist_motion_detector::MotionDetectorNodelet::onInit()");
    _node.reset(new MotionDetector(getPrivateNodeHandle()));
}

}	// namespace aist_depth_filter

PLUGINLIB_EXPORT_CLASS(aist_motion_detector::MotionDetectorNodelet,
		       nodelet::Nodelet);
